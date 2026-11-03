"""One broad-contact false merge through original segment preprocessing."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import ast,hashlib,json,os,sys,time
W=Path(__file__).resolve().parents[1];M=Path('/media/zhuyupei/SanStand')
N=M/'实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
O=N/'gen30_component_negative_provenance_r1'
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
path=W/'.runtime/prototype_gen30_measured_component.py';script_sha=sha(path)
tree=ast.parse(path.read_text());names={'geometry_audit','prototype','reference_for','synthetic_cases'}
defs=ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names],type_ignores=[])
assert len(defs.body)==len(names)
exec(compile(defs,str(path),'exec'),globals())
name,d,neighbor,reference,origin=next(row for row in synthetic_cases() if row[0]=='wide_diagonal_contact')
original_masks=[c.mask.copy() for c in d['result'].candidates]
result=table.segment_with_support_fallback(d['depth'],d['intrinsics'],d['transform'],d['config'],
 d['anchor'],d['last'],TEST_PLANE,prefer_current_support=True)
native=[dict(mask_pixels=int(np.count_nonzero(c.mask)),bbox=c.bbox,position_base=c.position_base,
 extent_m=c.extent_m,original_matching=bool(table._matching_candidates([c],d['anchor'],d['last'],d['config']))) for c in result.candidates]
d['result']=result;d['rgb_valid']=np.ones(d['depth'].shape,bool);cv2.setNumThreads(1)
audit,proposal,refined=prototype(d,reference,2001)
output=0 if refined is None else int(np.count_nonzero((refined.mask>0)&neighbor))
O.mkdir();cv2.imwrite(str(O/'rgb.png'),d['rgb']);cv2.imwrite(str(O/'known_neighbor.png'),neighbor.astype(np.uint8)*255)
for i,c in enumerate(result.candidates):cv2.imwrite(str(O/f'production_native_{i}.png'),c.mask)
if refined is not None:cv2.imwrite(str(O/'refined.png'),refined.mask)
report=dict(case=name,actual_production_segmentation=result.metrics,actual_native_candidates=native,
 matching_count=len(table._matching_candidates(result.candidates,d['anchor'],d['last'],d['config'])),
 original_fixture_native_pixel_counts=[int(np.count_nonzero(m)) for m in original_masks],
 known_neighbor_pixels=int(neighbor.sum()),known_neighbor_accepted_pixels=output,prototype=audit,
 classification='unsafe_false_merge_survives_original_segment_preprocessing' if output and audit['outcome']=='candidate_passed_gates_but_not_admitted' else 'counterexample_did_not_survive_original_segmentation',
 sources_sha256={str(m.__file__):sha(m.__file__) for m in (table,rgb,existing)},prototype_script_sha256=script_sha,
 fixture_dimensions_unchanged=True,one_case_only=True,original_morphology_and_4_neighbor_segmentation_used=True,
 no_ros_network_calls=True,no_motion_commands=True,production_or_tests_changed=False,production_deployed=False)
assert sha(path)==script_sha
(O/'result.json').write_text(json.dumps(report,indent=2,default=serial)+'\n');print(json.dumps(report,default=serial));print(O/'result.json',sha(O/'result.json'))
