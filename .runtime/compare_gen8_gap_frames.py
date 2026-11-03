from pathlib import Path
import json,dataclasses
from types import SimpleNamespace as NS
import numpy as np
from alicia_grasp_modes.observe_return_visual import measured_bilateral_view_gaps
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
O=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen8_contact_analysis')
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261009T023854.657870Z_44fae4d9c54c40c8a8ecf0969e9ca5d1')
d=json.loads((R/'keyframes_audits/6aa2e914a64e0bd50cf21f8452d46cb049bad61523c5f34d5e873165bcd6cc93.json').read_text());r=json.loads((O/'gain_gap_reconstruction.json').read_text());p=np.load(O/'fused_inputs.npz');g=d['replay_geometry'];geometry=NS(center_base=np.array(g['obb_center_base_m']),axes_base=np.array(g['R_base_obb']),size_xyz_m=np.array(g['obb_size_xyz_m']),support_normal_base=np.array(g['support_normal_base']));surface=FusedTargetSurface(TargetTrackIdentity(6,'g1-t6'),p['final_points'],p['final_view_indices'],tuple(r['fused_source_stamps_ns']));finger=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003);cfg=NS(max_inner_gap_m=.05,opening_fit_clearance_each_side_m=.0005,min_contact_band_points=12);camera=np.array(r['current_measured_gaps'][0]['initial_camera_origin_base_m']);raw=measured_bilateral_view_gaps(d['remote_diagnostics'],geometry,cfg,surface,finger,camera);report=dict(actual_prepare_supplement_frames='phase_reference stored fused surface combined with current_snapshot prepared.geometry and current camera',consistent_frames='inverse latest registration on fused points, combined with same current_snapshot geometry/camera',reference_to_current_transform=np.linalg.inv(p['latest_transform']).tolist(),actual_mixed_frame_gaps=raw,consistent_current_frame_gaps=r['current_measured_gaps'],gain_unaffected=True,no_ros_actions=True,diagnostic_only=True)
(O/'gap_frame_comparison.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
