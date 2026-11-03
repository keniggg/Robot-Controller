from pathlib import Path
import sys, json, importlib.util, inspect, types
import numpy as np
W=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision import regular_box_inference as box
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
s=importlib.util.spec_from_file_location('prototype',W/'.runtime/gen31_outer_rgb_rectangle_prototype.py');p=importlib.util.module_from_spec(s);s.loader.exec_module(p)
text=inspect.getsource(box.qualify_regular_box_top)
start=text.index('        visual = _visual_rectangle(');end=text.index('        size = ',start)
text=text[:start]+'''        outer = infer_outer_rgb_rectangle(color_bgr,mask,depth,scale,T,K,origin,tn,axes,pixel_size,rms)
        if not outer['ok']:
            return _failure('BOX_OUTER_RGB_UNRESOLVED',outer['reason'],dict(audit,outer_rgb_interval=outer['audit']))
        axes,lower,upper=outer['axes'],outer['lower'],outer['upper']
        coord=top@axes[:,:2]
        visual=None
'''+text[end:]
text=text.replace("        if (visual is None and fill < .88) or min(corner_coverage) < .5 or min(edge_coverage) < .7:","        if False:")
text=text.replace("        if visual is not None:\n            # Complete visual", "        if outer is not None:\n            # Complete visual")
text=text.replace("occupancy < (.65 if visual is not None else .88)","occupancy < .65")
start=text.index('        continuation, outside_valid = [], []');end=text.index('        edge_rms = ',start)
text=text[:start]+"        continuation,outside_valid=outer['continuation'],outer['outside_valid']\n"+text[end:]
start=text.index('        measured = np.asarray(')
text=text[:start]+"        edge_uncertainty=outer['edge_uncertainty']\n        yaw_uncertainty=outer['yaw_uncertainty']\n        edge_rms=outer['edge_rms']\n"+text[start:]
text=text.replace("contour_source='rgb_edges_on_measured_top_plane' if visual is not None else 'measured_depth_top')","contour_source='rgb_outer_joint_interval_on_measured_top_plane',outer_rgb_interval=outer['audit'])")
text=text.replace('model = dict(schema_version=1','model = dict(schema_version=2')
ns=dict(vars(box));ns['infer_outer_rgb_rectangle']=p.infer_outer_rgb_rectangle;ns['RegularBoxInference']=types.SimpleNamespace(from_evidence_dict=lambda d:d)
exec(compile(text,'<isolated_gen31_full_qualifier>','exec'),ns)
I=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z/verification/gen30_attempt1_closed_evidence_r1')
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261016T021226.502516Z_0494c8bc84a34a74933e8eae122dd1f4')
a=json.loads((R/'keyframes_audits/07a8ac807d6a7f0702ef09c8866949f4ce96f8379dc1421dc6ae203af6086135.json').read_text());g=a['replay_geometry'];geom=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g['support_inlier_ratio'],g['object_points_base_m'],g['source_mode']);x=np.load(I/'1792116802511774778_raw.npz');z=np.load(I/'first_observation_box_trace_arrays.npz')
r=ns['qualify_regular_box_top'](x['depth_raw'],x['object_mask'],z['K'],9.999999747378752e-05,z['T_base_optical'],geom,TargetTrackIdentity(4,'g1-t4'),1792116802511774778,color_bgr=x['color_bgr'])
out=dict(mode='offline_isolated_full_qualifier_counterfactual_no_model_validation_or_motion',ok=r.ok,code=r.code,audit=dict(r.audit),raw_model=r.model)
O=I.parent/'gen31_first_outer_candidate_counterfactual_r1';(O/'full_qualifier_counterfactual.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
