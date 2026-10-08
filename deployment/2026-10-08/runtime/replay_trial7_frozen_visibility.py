#!/usr/bin/env python3
"""Offline trial7 replay; reads archives, writes only the named local report.

Run after sourcing /opt/ros/noetic/setup.bash and devel/setup.bash for message
and FK imports. No ROS node, subscription, service, or command is created.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
for name in ('tools/diagnostics','src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src'):
    sys.path.insert(0,str(PROJECT/name))
from run_rgb_acquisition import rank_live_interest, ObservedViewHistory, NAMES
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from moveit_msgs.msg import RobotTrajectory


def pixels(vertices, transform, matrix):
    """Unclipped diagnostic projection, also reports outside-image failures."""
    camera=(np.asarray(vertices)-transform[:3,3]) @ transform[:3,:3]
    optical=np.c_[-camera[:,1],-camera[:,2],camera[:,0]]
    homogeneous=optical @ matrix.T
    values=homogeneous[:,:2]/homogeneous[:,2,None]
    return dict(vertices_px=values.tolist(),minimum_px=values.min(axis=0).tolist(),
        maximum_px=values.max(axis=0).tolist(),optical_depth_range_m=[float(optical[:,2].min()),float(optical[:,2].max())])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-root',type=Path,default=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log'))
    parser.add_argument('--output',type=Path,default=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/rgb_acquisition_run_20261004T060311Z/trial7_frozen_visibility_replay.json'))
    args=parser.parse_args()
    mount=Path('/media/zhuyupei/SanStand')
    if not mount.is_mount() or mount/'实验记录/Robot-Controller' not in args.output.resolve().parents:
        raise ValueError('SanStand report output required; no system disk fallback')
    root=args.log_root
    run_path=root/'rgb_acquisition_run_20261004T060311Z/result.json'
    first_dir=root/'rgb_acquisition_gateway_20261004T060320.112438Z_f52a5a1a'
    second_dir=root/'rgb_acquisition_gateway_20261004T060325.983836Z_2884f100'
    first_path=first_dir/'result.json'
    second_path=second_dir/'result.json'
    description_path=second_dir/'robot_description.urdf'
    run=json.loads(run_path.read_text())
    first=json.loads(first_path.read_text())['frames']['initial']['interest']
    submission=json.loads(second_path.read_text())['frames']['submission']['interest']
    description=description_path.read_text()
    fk=SerialUrdfFk(description,NAMES)
    interest=run['interest']; ledger=run['requests'][0]['ledger']
    options=dict(selection=interest['source_context']['selection'],description=description,
        joint_min=fk.limits[:,0],joint_max=fk.limits[:,1])
    history=ObservedViewHistory()
    rank_live_interest(first,now_ns=first['source_stamp_ns']+100000000,view_history=history,**options)
    history.bind_success(ledger)
    started=time.monotonic()
    ranking,candidates=rank_live_interest(interest,now_ns=interest['source_stamp_ns']+100000000,
        view_history=history,reference_ledger=ledger,**options)
    elapsed=time.monotonic()-started
    frozen=json.loads(ledger['reference_volume_json'])
    mount=np.asarray(interest['source_context']['tool_camera']); matrix=np.asarray(interest['interest_volume']['camera_matrix'])
    plan=RobotTrajectory().deserialize((second_dir/'trajectory.rosmsg').read_bytes())
    goal=np.asarray(plan.joint_trajectory.points[-1].positions)
    endpoint=fk(goal) @ mount
    old_selected=run['requests'][1]['candidate_id']
    verdict=next(row for row in ranking['rejected'] if row['candidate_id']==old_selected)
    assert verdict.get('frozen_reference_visibility_rejected') is True
    assert all(row['candidate_id']!=old_selected for row in ranking['ranked'])
    result=dict(version=1,offline_only=True,hardware_commands_sent=False,execution_authority=False,
        reference_id=ledger['reference_id'],reference_observation_set_sha256=ledger['reference_observation_set_sha256'],
        selection=options['selection'],source_id=interest['source_id'],source_stamp_ns=interest['source_stamp_ns'],
        observation_history=history.snapshot(),old_selected=old_selected,selected_rejection=verdict,
        nominal_legal_candidate_count=len(ranking['ranked']),ranking_seconds=elapsed,
        original_selected_fk_proposal_projection=pixels(frozen['vertices_base_m'],np.asarray(candidates[old_selected].T_base_camera),matrix),
        executed_plan_endpoint_original_projection=pixels(frozen['vertices_base_m'],endpoint,matrix),
        executed_plan_endpoint_ranking_frame_volume_projection=pixels(interest['interest_volume']['vertices_base_m'],endpoint,matrix),
        executed_plan_endpoint_submission_frame_volume_projection=pixels(submission['interest_volume']['vertices_base_m'],endpoint,matrix),
        limits=dict(roi_margin_px=ranking['roi_margin_px'],working_depth_range_m=interest['interest_volume']['working_depth_range_m']),
        ranking=ranking,
        limitations=['nominal endpoints only; no continuous or tracking-error visibility certification',
                     'legal view candidates do not establish reachability/collision-free motion or information recovery',
                     'the old episode cannot be resumed merely because this offline replay has valid candidates'],
        input_files={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in (
            run_path,first_path,second_path,description_path,second_dir/'trajectory.rosmsg')},
        implementation_sha256={name:hashlib.sha256((PROJECT/name).read_bytes()).hexdigest() for name in (
            'tools/diagnostics/run_rgb_acquisition.py',
            'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_observation_views.py')})
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(report=str(args.output),rejected=old_selected,
        reason=verdict['reason'],legal_candidates=[row['candidate_id'] for row in ranking['ranked']],
        ranking_seconds=elapsed,sha256=hashlib.sha256(args.output.read_bytes()).hexdigest()),indent=2))


if __name__=='__main__':
    main()
