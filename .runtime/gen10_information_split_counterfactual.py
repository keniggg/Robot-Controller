#!/usr/bin/env python3
"""Frozen gen10 information-only uncertainty counterfactual, no ROS calls."""
from pathlib import Path
import hashlib
import json
import numpy as np
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_grasp_modes.reachable_observation_views import generate_reachable_observation_views
from alicia_grasp_modes.observe_return_visual import directed_missing_side_rotations

EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
ROOT=EXT/'ros_log/full_latest_ros_20261009T003407Z/gen10_offline_ik'
RECORD=EXT/'grasp_records/20261009T031019.431802Z_cf2238ea63164fd2a86f610925c32457'
AUDIT=RECORD/'keyframes_audits/18452e4dea5750147f6da04b6122b9f605e6b39e98ef10c966855962b624f3d3.json'

def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    raw=AUDIT.read_bytes();audit=json.loads(raw)
    source=json.loads((ROOT/'frozen_source/source.json').read_text())
    with np.load(str(ROOT/'frozen_source/source.npz')) as arrays:
        source.update({key:arrays[key].copy() for key in arrays.files})
    geo=audit['replay_geometry']
    source['geometry']=dict(support_normal_base=geo['support_normal_base'],
        support_offset_m=geo['support_offset_m'],R_base_obb=geo['R_base_obb'],
        obb_size_xyz_m=geo['obb_size_xyz_m'])
    original=audit['observation_failure_diagnostics']['joint_space_fk_search']['audit']
    model=Path(source['source_provenance']['model_path']).read_text()
    fk=SerialUrdfFk(model,['Joint%d'%i for i in range(1,7)])
    assert fk.model_sha256==original['model_sha256']
    q0=np.asarray(original['current_joint_positions'])
    assert np.array_equal(q0,source['current_joint_positions'])
    initial_camera=fk(q0) @ source['tool_from_camera']
    gaps=audit['observe_return']['measured_missing_sides']
    prior=json.loads((RECORD/'keyframes_audits/88ea33aa28b5406fa05c8bec8d0b1d8c3e6173ddada0c34144853578f1dacdd2.json').read_text())
    profiles=prior['remote_diagnostics']['tabletop_geometry']['adaptive_stage_profiles']
    combined=max(float(row['depth_uncertainty_m']) for row in profiles)
    perception=max(float(row['depth_uncertainty_m'])-float(row['execution_position_error_m']) for row in profiles)
    assert abs(perception-.003)<1e-12 and combined>perception
    common=dict(source=source,current_joint_positions=q0,fk=fk,
        gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003),
        opening_width_m=.05,gaps=gaps,joint_error_bounds_rad=original['joint_error_bounds_rad'],
        max_seconds=2.,max_directions=2,proposal_image_reserve_px=4.)
    report=dict(offline=True,ros_calls=False,execution_authority=False,
        purpose='Change only predicted information resolution; contact profiles, FK limits, following and all physical endpoint gates retain original values.',
        source_audit_sha256=hashlib.sha256(raw).hexdigest(),model_sha256=fk.model_sha256,
        frozen_source_sha256=hashlib.sha256((ROOT/'frozen_source/source.json').read_bytes()).hexdigest(),
        original_combined_uncertainty_m=combined,perception_uncertainty_m=perception,
        unchanged_execution_position_error_m=combined-perception,results=[])
    for label,uncertainty in [('combined',combined),('perception_only',perception)]:
        candidates,result=generate_reachable_observation_views(**common,depth_uncertainty_m=uncertainty)
        rotations=directed_missing_side_rotations(gaps,initial_camera[:3,:3],
            np.array(geo['support_normal_base']),uncertainty,
            float(np.linalg.norm(np.array(geo['obb_center_base_m'])-initial_camera[:3,3])))
        entry=dict(label=label,uncertainty_m=uncertainty,candidates=candidates,audit=result,
            directed_rotation_degrees=[metadata['camera_rotation_offset_deg'] for _,metadata in rotations])
        report['results'].append(entry)
        for index,candidate in enumerate(candidates):
            path=ROOT/('information_%s_candidate%d.json'%(label,index))
            candidate=dict(candidate,source_audit_sha256=report['source_audit_sha256'],
                diagnostic_information_uncertainty_m=uncertainty)
            with path.open('x') as output:json.dump(candidate,output,indent=2,allow_nan=False)
        print(json.dumps(dict(label=label,candidates=len(candidates),elapsed=result['elapsed_sec'],
            rotations=entry['directed_rotation_degrees'],reasons=[r.get('reason') for r in result['directions']])),flush=True)
    with (ROOT/'information_split_result.json').open('x') as output:
        json.dump(report,output,indent=2,allow_nan=False)

if __name__=='__main__':main()
