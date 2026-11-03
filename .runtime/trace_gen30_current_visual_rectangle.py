"""Trace original visual rectangle gates for exactly three new passive sources.

No production replacement, threshold alternatives, ROS or hardware access.
"""
import ast
from collections import Counter
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
C=M/'实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z'
I=C/'verification/gen30_current_passive_initial_r1'
O=C/'verification/gen30_current_visual_trace_r1'
assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
assert not O.exists()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1', PYTHONDONTWRITEBYTECODE='1',
                  OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
for name in ('src/alicia_flexible_grasp_supervisor/src', 'devel/lib/python3/dist-packages'):
    sys.path.insert(0,str(W/name))
import numpy as np
from alicia_flexible_grasp.vision import regular_box_inference as box
from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_top
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample,fuse_stable_samples
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def arrsha(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()
def serial(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)

inputs={str(I/'result.json'):sha(I/'result.json')}
prior=json.loads((I/'result.json').read_text())
ids=tuple(prior['selected_source_stamps_ns'])
assert ids==(1792111336719579458,1792111337052526473,1792111337386243343)
source_names=[inspect.getsourcefile(m) for m in
    (box,sys.modules[fuse_stable_samples.__module__],sys.modules[estimate_object_geometry.__module__])]
sources={name:sha(name) for name in source_names}
assert all(prior['source_sha256'][name]==digest for name,digest in sources.items())
identity=TargetTrackIdentity(**prior['diagnostic_identity'])
arrays=[];samples=[]
for ns,row in zip(ids,prior['rows']):
    path=I/('%d_arrays.npz'%ns);inputs[str(path)]=sha(path)
    with np.load(path,allow_pickle=False) as archive:
        a={key:archive[key].copy() for key in archive.files}
    assert all(arrsha(a[key])==digest for key,digest in row['array_sha256'].items())
    assert int(a['source_stamp_ns'])==ns
    boxxy=tuple(row['original_rgb_refinement']['bbox'])
    message=NS(detected=True,bbox_x=boxxy[0],bbox_y=boxxy[1],bbox_width=boxxy[2],bbox_height=boxxy[3])
    q=np.asarray(row['accepted_joint_bracket']['before']['positions_rad'])
    samples.append(RgbdSample(a['color_bgr'],a['depth_raw'],a['object_mask'],boxxy,
        message,ns/1e9,'camera_link',q,ns,identity.epoch,identity))
    arrays.append(a)
scale=float(arrays[-1]['depth_scale_m']);k=arrays[-1]['intrinsics']
K=NS(fx=k[0],fy=k[1],cx=k[2],cy=k[3],width=arrays[-1]['depth_raw'].shape[1],height=arrays[-1]['depth_raw'].shape[0])
optical_to_link=np.eye(4);optical_to_link[:3,:3]=[[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]]
T=arrays[-1]['T_base_camera_link']@optical_to_link
snapshot=fuse_stable_samples(samples,**prior['downstream']['snapshot_arguments'])
assert snapshot.ok and snapshot.sample_stamp_ns==ids
geometry=estimate_object_geometry(snapshot.depth_raw,snapshot.target_depth_raw,snapshot.object_mask,
    snapshot.bbox,K,scale,T,snapshot.source_mode,**prior['downstream']['geometry_arguments'])
assert geometry.ok and len(geometry.object_points_base)==prior['downstream']['object_cloud_points']
for key,recorded in [('center_base','geometry_center_base'),('axes_base','geometry_axes_base'),('size_xyz_m','geometry_size_m')]:
    np.testing.assert_array_equal(getattr(geometry,key),prior['downstream'][recorded])

# Reuse only the previous read-only line tracer's function AST. Importing its
# module would rerun a historical reconstruction, so no module is imported.
tracer_path=W/'.runtime/trace_gen30_initial_visual_rectangle.py'
inputs[str(tracer_path)]=sha(tracer_path)
tracer_text=tracer_path.read_text();tree=ast.parse(tracer_text)
function=next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name=='qualify')
text=ast.get_source_segment(tracer_text,function)
old="        return trace if frame.f_code is qualify_regular_box_top.__code__ else None"
new="""            for key in ('origin','tn','axes','lower','upper','pixel_size','top_height','corners_base','image_corners'):
                if key in v:
                    local[key]=v[key].tolist() if isinstance(v[key],np.ndarray) else v[key]
        return trace if frame.f_code is qualify_regular_box_top.__code__ else None"""
assert text.count(old)==1
text=text.replace(old,new)
exec(compile(text,'<readonly_visual_line_tracer>','exec'),globals())
rows=[]
for ns,a in zip(ids,arrays):
    result=qualify(lambda a=a,ns=ns:qualify_regular_box_top(a['depth_raw'],a['object_mask'],K,scale,
        a['T_base_camera_link']@optical_to_link,geometry,identity,ns,color_bgr=a['color_bgr']))
    rows.append(dict(source_stamp_ns=ns,qualification=result))
assert rows[-1]['qualification']['code']==prior['downstream']['box_code']
for key,value in rows[-1]['qualification']['audit'].items():
    assert prior['downstream']['box_audit'][key]==value,(key,value)
assert all(sha(path)==digest for path,digest in sources.items())
assert all(sha(path)==digest for path,digest in inputs.items())
O.mkdir()
report=dict(classification='original_current_three_source_visual_branch_trace',recorded_ns=time.time_ns(),
    rows=rows,input_sha256=inputs,source_sha256=sources,script_sha256=sha(__file__),
    source_unchanged=True,input_unchanged=True,production_modified=False,
    no_ros_network_calls=True,no_hardware_access=True,threshold_changes=False,
    runtime_target_identity_used=False,execution_authority=False,
    current_fused_geometry_reproduced_exactly=True,
    last_full_qualifier_result_matches_previous_passive_analysis=True,
    limitations=prior['limitations']+['First two pixel qualifiers use the same reconstructed fused measured geometry; no live planning request is reproduced.'])
(O/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,default=serial,allow_nan=False)+'\n')
print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),
    rows=[dict(source_stamp_ns=r['source_stamp_ns'],code=r['qualification']['code'],
        trace=r['qualification']['return_local_diagnostics']['visual_rectangle_trace']) for r in rows]),ensure_ascii=False,default=serial))
