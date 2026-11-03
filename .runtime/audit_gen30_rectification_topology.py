"""Finite historical map/topology audit. SDK arithmetic only; no device or ROS I/O."""
from pathlib import Path
from collections import Counter
import hashlib,json,os,sys,time
W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
N=M/'实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
I=N/'gen29_attempt1_shadow_frames';O=N/'gen30_rectification_topology_r1'
assert M.is_mount() and not O.exists() and os.environ['ROS_MASTER_URI']=='http://127.0.0.1:1'
sys.path[:0]=[str(W/'.runtime/realsense-2.56.5'),str(W/'src/alicia_flexible_grasp_supervisor/src'),str(W/'src/alicia_grasp_modes/src')]
import cv2,numpy as np,pyrealsense2 as sdk
from alicia_flexible_grasp.vision import rgbd_rectification as rect
from alicia_grasp_modes import tabletop as table
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def serial(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 raise TypeError(type(v).__name__)
inputs={}
def read(p):inputs[str(p)]=sha(p);return json.loads(p.read_text())
previous=read(N/'gen30_component_entry_and_edges_r1/result.json')
meta=read(I/'mask_contract_review_r2/result.json');cfg=table.Config(**meta['effective_config'])
rows=[];maps={};cv2.setNumThreads(1)
for prior in previous['actual_diagonal_edges']:
 ns=prior['source_stamp_ns'];p=I/f'source_{ns}'
 camera=read(p/'metadata.json')['camera_parameters'];record=rect.rectification_contract(camera)
 rect.validate_rectification_contract(record)
 assert camera['runtime_profile']['color_intrinsics']==record['source_intrinsics']
 assert camera['runtime_profile']['geometry_intrinsics']==record['output_intrinsics']
 assert camera['runtime_profile']['sdk_version']==sdk.__version__==record['sdk_version']
 key=record['digest']
 if key not in maps:
  wall,cpu=time.perf_counter(),time.process_time()
  mapping=rect.PairedPinholeRectification(record['source_intrinsics'],sdk=sdk)
  assert mapping.contract==record,'Exact historical SDK/calibration/map contract changed'
  inverse=np.full((mapping.height,mapping.width),-1,np.int64)
  inverse.ravel()[mapping.target_indices]=mapping.source_indices
  maps[key]=(mapping,inverse,dict(wall_sec=time.perf_counter()-wall,cpu_sec=time.process_time()-cpu))
 mapping,inverse,timing=maps[key]
 path=p/'raw_arrays.npz';inputs[str(path)]=sha(path);a=dict(np.load(path,allow_pickle=False))
 raw=a['depth_raw'];depth=raw.astype(np.float32)*float(a['depth_scale_m'])
 assert raw.shape==inverse.shape and np.array_equal(a['intrinsics'],[record['output_intrinsics'][k] for k in ('fx','fy','cx','cy')])
 assert not np.any(raw[inverse<0]),'Unmapped canvas carries a measurement'
 path=N/'gen30_measured_component_prototype_r1'/f'{ns}_whole_raw.png';inputs[str(path)]=sha(path);occupied=cv2.imread(str(path),0)>0
 assert not np.any(occupied&(inverse<0))
 components=table._depth_components(occupied.astype(np.uint8),depth,cfg.min_object_points,cfg.depth_edge_m,connectivity=4)
 labels=np.zeros(raw.shape,np.int32)
 for i,m in enumerate(components):labels[m]=i+1
 edges=[];counts=Counter();adjacency=Counter();graph={i:set() for i in range(len(components))}
 for edge in prior['edges']:
  y,x=edge['first_yx'];y2,x2=edge['second_yx'];mid=[(y,x2),(y2,x)]
  assert raw[y,x]>0 and raw[y2,x2]>0
  first=int(inverse[y,x]);second=int(inverse[y2,x2]);assert min(first,second)>=0
  fy,fx=divmod(first,mapping.source_width);sy,sx=divmod(second,mapping.source_width)
  manhattan=abs(fy-sy)+abs(fx-sx)
  depth_delta=abs(float(depth[y,x])-float(depth[y2,x2]));assert depth_delta<=cfg.depth_edge_m
  categories=[]
  for my,mx in mid:
   assert raw[my,mx]==0
   categories.append('structurally_unmapped_canvas' if inverse[my,mx]<0 else 'mapped_source_depth_invalid')
  counts[' + '.join(sorted(categories))]+=1
  adjacency[str(manhattan)]+=1
  if manhattan==1:
   aidx,bidx=int(labels[y,x]-1),int(labels[y2,x2]-1);graph[aidx].add(bidx);graph[bidx].add(aidx)
  edges.append(dict(edge,orthogonal_provenance=categories,orthogonal_source_indices=[int(inverse[yy,xx]) for yy,xx in mid],
   endpoint_source_yx=[[fy,fx],[sy,sx]],source_grid_manhattan=manhattan,
   original_source_four_neighbours=manhattan==1,current_depth_delta_m=depth_delta))
 unseen=set(graph);groups=[]
 while unseen:
  stack=[unseen.pop()];part=set(stack)
  while stack:
   for nxt in graph[stack.pop()]:
    if nxt not in part:part.add(nxt);unseen.discard(nxt);stack.append(nxt)
  groups.append(sorted(part))
 # Compare the complete occupied graph directly on the original pre-rectified
 # aligned-color raster, retaining only the same current measured samples.
 source_occupied=np.zeros((mapping.source_height,mapping.source_width),np.uint8)
 source_depth=np.zeros(source_occupied.shape,np.float32)
 source_occupied.ravel()[mapping.source_indices]=occupied.ravel()[mapping.target_indices]
 source_depth.ravel()[mapping.source_indices]=depth.ravel()[mapping.target_indices]
 source_components=table._depth_components(source_occupied,source_depth,cfg.min_object_points,cfg.depth_edge_m,connectivity=4)
 rows.append(dict(source_stamp_ns=ns,exact_contract_equal=True,contract_digest=key,
  depth_forward_map_sha256=record['depth_forward_map_sha256'],rgb_inverse_map_sha256=record['rgb_inverse_map_sha256'],
  sdk_version=sdk.__version__,source_intrinsics=record['source_intrinsics'],
  camera_geometry_activation_stamp_ns=camera['runtime_profile']['geometry_activation_stamp_ns'],
  reconstruction_time_once_per_contract=timing,diagonal_count=len(edges),orthogonal_provenance_counts=dict(counts),
  source_grid_manhattan_counts=dict(adjacency),native_connectivity_from_source4_edges=groups,
  whole_component_source4_counts=sorted(int(c.sum()) for c in source_components),
  current_measured_mask_unchanged=True,no_depth_filled=True,edges=edges))
report=dict(rows=rows,input_sha256=inputs,script_sha256=sha(__file__),
 source_sha256={str(m.__file__):sha(m.__file__) for m in (rect,table)},
 sdk_module_path=sdk.__file__,sdk_module_sha256=sha(sdk.__file__),
 preliminary_import_note='Default user-site pyrealsense2 had no __version__; audit stopped before reconstruction, then used the same isolated 2.56.5 numerical SDK as production.',
 distinct_historical_contracts_reconstructed=len(maps),no_ros_network_calls=True,no_device_io=True,no_motion=True,
 production_or_tests_changed=False,production_deployed=False,
 limitations=['Source raster is the SDK aligned-to-color raster immediately before paired pinhole rectification; this is not the unaligned native depth sensor raster.',
 'Source adjacency restores sample topology only. It does not prove single-object identity, depth coverage between samples, downstream shape qualification, freshness, or movement permission.',
 'All output-depth zeros remain unknown: structural canvas gaps and mapped invalid source samples are distinguished, never filled.'])
assert all(sha(p)==h for p,h in inputs.items())
O.mkdir();(O/'result.json').write_text(json.dumps(report,indent=2,default=serial,allow_nan=False)+'\n')
for row in rows:print(json.dumps({k:v for k,v in row.items() if k not in ('edges','source_intrinsics')},default=serial),flush=True)
print(O/'result.json',sha(O/'result.json'),flush=True)
