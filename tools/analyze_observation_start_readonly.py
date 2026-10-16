#!/usr/bin/env python3
"""Recompute frozen hold/goal tracking envelopes from a CLOSED grasp record.

No ROS node, parameter server, publisher, service or device connection. This
report does not authorize a replacement movement or reduced tracking bounds.
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import re

import numpy as np
import rosbag

from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, SerialUrdfFk, observation_following_support_bound,
    observation_tracking_error_bounds,
)


def analyze(bag_path, audit_path, urdf_path, prior_fixture=None):
    payload = Path(audit_path).read_bytes()
    audit = json.loads(payload)
    names = ['Joint%d'%i for i in range(1,7)]
    model = SerialUrdfFk(Path(urdf_path).read_text(), names)
    gripper = GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),
                              np.array([.1175,.155,.0774]),.003)
    constraints = {n:{'trajectory':.12,'goal':.035} for n in names}
    errors = observation_tracking_error_bounds(constraints,names)
    counts, ranges = collections.Counter(), {}
    scene, hold, measured, opening = None,None,None,None
    with rosbag.Bag(str(bag_path)) as bag:
        for topic,msg,_ in bag.read_messages(topics=[
                '/grasp_6d/plan_enriched','/alicia_controller/command',
                '/alicia_d/accepted_joint_states',
                '/alicia_controller/follow_joint_trajectory/goal']):
            counts[topic] += 1
            if topic=='/grasp_6d/plan_enriched' and msg.valid and msg.plan_id==audit['plan_id']:
                scene = FrozenObservationScene.from_plan(msg)
            elif topic=='/alicia_controller/command' and len(msg.points)==1:
                by_name=dict(zip(msg.joint_names,msg.points[0].positions))
                hold=[by_name[n] for n in names]
            elif topic=='/alicia_d/accepted_joint_states':
                by_name=dict(zip(msg.name,msg.position))
                measured=[by_name[n] for n in names]; opening=by_name['right_finger']
                for n,v in by_name.items():
                    limits=ranges.setdefault(n,[v,v]);limits[0]=min(limits[0],v);limits[1]=max(limits[1],v)
    if scene is None or hold is None or measured is None:
        raise ValueError('closed bag lacks exact committed scene, hold or accepted feedback')
    def bound(q, normal=scene.normal, offset=scene.offset, gap=opening):
        return observation_following_support_bound(model,q,errors,normal,offset,gripper,gap)
    result=dict(read_only=True,grasp_success_proven=False,replacement_path_executed=False,
        plan_id=scene.plan_id,source_ns=scene.source_ns,audit_sha256=hashlib.sha256(payload).hexdigest(),
        model_sha256=model.model_sha256,controller_constraints_assumption=constraints,
        recorded_message_counts=dict(counts),accepted_min_max=ranges,
        accepted_start=bound(measured),successful_sdk_hold_start=bound(hold),
        selected_search={k:v for k,v in audit['selected']['observation_orientation_search'].items()
                         if k!='variant_results'},candidate_endpoints=[])
    for row in audit['selected']['observation_orientation_search']['variant_results']:
        old=row.get('following_support',{})
        if 'goal_joint_positions_rad' not in old:continue
        if old['robot_description_sha256']!=model.model_sha256:
            raise ValueError('recorded candidate model differs from replay model')
        result['candidate_endpoints'].append(dict(roll=row['camera_roll_offset_deg'],
            tilt=row['camera_tilt_offset_deg'],distance=row['camera_target_distance_m'],
            lower_bound_duration=row.get('execution_duration_lower_bound_sec'),
            endpoint_support=bound(old['goal_joint_positions_rad'],gap=.05)))
    if prior_fixture:
        previous=json.loads(Path(prior_fixture).read_text())
        if previous['model_sha256']!=model.model_sha256:raise ValueError('prior model mismatch')
        result['previous_start_comparison']=dict(plan_id=previous['plan_id'],
            support=bound(previous['executed'][0]['hold'],np.array(previous['normal']),
                          previous['offset'],previous['opening']))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('bag','audit','urdf','output'):parser.add_argument('--'+name,required=True)
    parser.add_argument('--prior-fixture')
    args=parser.parse_args()
    report=analyze(args.bag,args.audit,args.urdf,args.prior_fixture)
    with open(args.output,'x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps({k:report[k] for k in ('plan_id','successful_sdk_hold_start',
                                          'previous_start_comparison') if k in report},indent=2))


if __name__=='__main__':main()
