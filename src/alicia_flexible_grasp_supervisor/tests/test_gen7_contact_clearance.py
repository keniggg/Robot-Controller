"""Recorded gen7 start, replanned paths; no live ROS or physical success claims."""
import base64
import json
from pathlib import Path

import numpy as np
import pytest
from moveit_msgs.msg import RobotTrajectory
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, SerialUrdfFk, ObservationPathError,
    validate_observation_trajectory,
)
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry


def recorded():
    x=json.loads((Path(__file__).parent/'fixtures/gen7_contact_clearance_20260926.json').read_text())
    # The 2026-09-26 fixture predates the appended contact_geometry_evidence
    # string. Decode its recorded fields with an explicitly empty extension;
    # this is a geometry replay, never a current executable contact plan.
    wire = base64.b64decode(x['frozen_contact_plan_wire_base64'])
    p=Grasp6DPlan();p.deserialize(wire + b'\x00\x00\x00\x00')
    fk=SerialUrdfFk(x['robot_description'],ARM_NAMES)
    c=x['gripper_geometry']
    g=GripperGeometry(c['max_inner_gap_m'],c['width_safety_margin_per_side_m'],
                     np.asarray(c['finger_box_xyz_m']),np.asarray(c['palm_box_xyz_m']),.003)
    return x,p,fk,g,FrozenObservationScene.from_contact_plan(p)


def trajectory(row):
    p=RobotTrajectory();p.deserialize(base64.b64decode(row['wire_base64']))
    return p


def test_recorded_boundary_replan_remains_rejected_by_unchanged_3mm_guard():
    x,_,fk,g,scene=recorded()
    tr=trajectory(x['boundary_goal_replan']['paths']['nominal'])
    with pytest.raises(ObservationPathError,match='clearance'):
        validate_observation_trajectory(tr,scene,fk,g,x['measured_opening_m'],
                                        tr.joint_trajectory.points[0])


@pytest.mark.parametrize('kind',['nominal','shifted'])
def test_positive_candidate_clearance_proves_full_replanned_path(kind):
    x,plan,fk,g,scene=recorded()
    row=x['positive_clearance_goal_replan']
    assert row['hypothetical_source_goal_clearance_margin_m']==.0005
    assert not row['motion_executed']
    tr=trajectory(row['paths'][kind])
    audit=validate_observation_trajectory(tr,scene,fk,g,x['measured_opening_m'],
                                         tr.joint_trajectory.points[0])
    assert audit['certified_minimum_clearance_m'] >= .003
    if kind=='nominal':
        goal=np.array([plan.poses[2].position.x,plan.poses[2].position.y,plan.poses[2].position.z])
        np.testing.assert_allclose(fk(tr.joint_trajectory.points[-1].positions)[:3,3],
                                   goal+.0005*scene.normal,atol=1e-7)
