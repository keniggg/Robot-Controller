"""Replay checked alternatives from a real attempt; never execution authority."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
from test_remote_modes import configured, select_unknown, base


DATA = json.loads((Path(__file__).parent/'fixtures/unknown_execution_choice_20260928.json').read_text())


def candidates(node):
    node.near_field_planning_active = True
    node.gripper_tool_jaw_axis = 'y'
    node._stable_variant_runtime = {}
    result = []
    for row in DATA['candidates']:
        g = row['geometry']; T = np.array(row['grasp_transform'])
        pose = base.make_pose_stamped('base_link', T[:3,3], base.quaternion_from_matrix(T))
        c = NS(track_id=row['track'], variant_index=row['variant'], pre_moveit_score=0.,
            required_open_width_m=row['required_open_width_m'],
            moveit_result=base.MoveItResult(**row['result']))
        assert row['mujoco_passed'] and c.moveit_result.reachable
        node._stable_variant_runtime[(c.track_id,c.variant_index)] = dict(
            prepared=NS(geometry=NS(axes_base=np.array(g['axes_base']),size_xyz_m=np.array(g['size_xyz_m']))),
            sequence=NS(grasp=pose),final_registered_geometry_gate={'required_open_width_m':c.required_open_width_m})
        result.append(c)
    return result


def test_recorded_fast_path_beats_178_degree_wrist_reversal(configured):
    node,params=configured;select_unknown(params);assert node.reset_mode_cb(None).success
    rows=candidates(node);assert len(rows)==5
    frozen=deepcopy(DATA)
    selected=min(rows,key=node._direct_near_field_execution_rank_key)
    assert (selected.track_id,selected.variant_index)==(8,0)
    assert node._direct_near_field_execution_rank_key(selected)[0]==pytest.approx(80.798)
    slow=next(c for c in rows if (c.track_id,c.variant_index)==(2,0))
    assert node._direct_near_field_execution_rank_key(slow)[0]==pytest.approx(158.087)
    assert selected.moveit_result.joint_max_delta_rad < 1.56
    assert slow.moveit_result.joint_max_delta_rad > 3.10
    assert DATA==frozen


@pytest.mark.parametrize('reason',['strict','execution_duration_lower_bound_sec=nan hardware_limiting_joint=Joint3',
    'execution_duration_lower_bound_sec=0 hardware_limiting_joint=Joint3',
    'execution_duration_lower_bound_sec=1'])
def test_unknown_missing_duration_cannot_win_by_narrower_width(configured,reason):
    from dataclasses import replace
    node,params=configured;select_unknown(params);assert node.reset_mode_cb(None).success
    rows=candidates(node);fast=rows[-1];unknown=rows[0]
    unknown.moveit_result=replace(unknown.moveit_result,reason=reason)
    assert min([unknown,fast],key=node._direct_near_field_execution_rank_key) is fast
