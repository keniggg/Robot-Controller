#!/usr/bin/env python3
"""Read archived raw-wire RGB episode geometry and re-run gateway FOV only.

No ROS node/init, params, publishers, services, or hardware. JSON goes to stdout;
caller owns the actual mounted experiment output path.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from moveit_msgs.msg import RobotTrajectory
from alicia_flexible_grasp.robot.rgb_acquisition_gateway import RgbAcquisitionGateway
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk, wire_digest, ObservationPathError
from alicia_flexible_grasp.robot.observation_view_guard import validate_observation_target_trajectory
from alicia_flexible_grasp.robot.observation_view_limits import effective_observation_view_limits

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('artifact',type=Path)
args=parser.parse_args();directory=args.artifact
record_path=directory/'result.json'
record=json.loads(record_path.read_text());frame=record['frames']['submission']
plan=RobotTrajectory();plan.deserialize((directory/record['trajectory_file']).read_bytes())
xml=(directory/'robot_description.urdf').read_text()
model=SerialUrdfFk(xml,tuple(plan.joint_trajectory.joint_names))
if wire_digest(plan)!=record['trajectory_sha256'] or model.model_sha256!=record['robot_description_sha256']:
    raise ValueError('archive model or final trajectory hash mismatch')
for slot,data in record['frames'].items():
    if hashlib.sha256((directory/data['source_file']).read_bytes()).hexdigest()!=data['source_file_sha256']:
        raise ValueError('archive exact RGB-D source file mismatch: '+slot)
node=RgbAcquisitionGateway();episode=deepcopy(record['outcome']['episode'])
node._rgb_install_reference_visibility(episode)
context=frame['source_context'];camera=context['camera']
captured=dict(source_context=context,reference_id=episode['reference_id'],interest=frame['interest'],
    volumes=tuple(SimpleNamespace(vertices_base_m=row['vertices_base_m']) for row in frame['interest']['interest_volumes']),
    limits=effective_observation_view_limits(context['observation_view_parameters']))
desired=SimpleNamespace(positions=record['audit']['bridge_start_joint_positions_rad'],velocities=[0.]*6,accelerations=[0.]*6)
current=[]
for volume in captured['volumes']:
    current.append(validate_observation_target_trajectory(plan,model,np.asarray(context['tool_camera']),
        volume.vertices_base_m,[camera[k] for k in ('fx','fy','cx','cy')],camera['width'],camera['height'],
        desired,**captured['limits']))
report=dict(artifact=str(directory),result_sha256=hashlib.sha256(record_path.read_bytes()).hexdigest(),
    trajectory_sha256=wire_digest(plan),model_sha256=model.model_sha256,
    current_volumes_all_pass=all(item['ok'] for item in current),current_member_audits=current,
    reference_binding=node._rgb_reference_visibility(captured),
    source_fk_tf_max_abs=float(np.max(np.abs(model(frame['interest']['q_source'])@np.asarray(context['tool_camera'])
        -np.asarray(frame['interest']['T_base_camera'])))),
    motion_executed=False,ros_node_initialized=False)
try:
    report['gateway_fov']=node._rgb_member_visibility(captured,plan,model,desired)
    report['gateway_rejected']=False
except ObservationPathError as exc:
    report.update(gateway_rejected=True,reason=str(exc),rejection_audit=exc.audit)
print(json.dumps(report,ensure_ascii=False,indent=2))
