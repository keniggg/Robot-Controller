#!/usr/bin/env python3
"""CLOSED-bag initial evidence and explicitly hypothetical response replay.

No ROS node, service, publisher, live TF or hardware access. Simulated response
is NOT evidence that this arm can compensate its offset, nor motion clearance.
"""
import argparse
from copy import deepcopy
import json
import math

import numpy as np
import rosbag
import yaml

from alicia_flexible_grasp.robot.stationary_following import (
    ARM_NAMES, SerialUrdfFk, stationary_following_error,
)
from alicia_flexible_grasp.robot.endpoint_correction import EndpointCorrection


def replay(fk, initial, responsive=True):
    correction = EndpointCorrection(fk, initial)
    sample = deepcopy(initial)
    results = []
    try:
        for i in range(7):
            code, step, evidence = correction.evaluate(sample)
            results.append(dict(evidence, code=code))
            if step is None:
                return {'hypothesis': 'all_axes_repeatable_fixed_bias', 'results': results}
            results[-1]['proposed_delta_counts'] = [b-a for a,b in zip(
                step.baseline_counts, step.target_counts)]
            completed = initial['stamp_sec'] + 3*i + 1.
            correction.committed(step, completed)
            delta = np.asarray(step.positions) - sample['sdk_positions_rad']
            sample['sdk_positions_rad'] = list(step.positions)
            if responsive:
                sample['accepted_positions_rad'] = (np.asarray(sample['accepted_positions_rad']) + delta).tolist()
            sample['stamp_sec'] = completed + 1.
            sample['sdk_stamp_ns'] = int((completed + .95) * 1e9)
            sample['accepted_stamp_ns'] = sample['sdk_stamp_ns']
            sample['stationary_window_end_ns'] = sample['sdk_stamp_ns']
            sample['stationary_window_start_ns'] = sample['sdk_stamp_ns'] - 300_000_000
            sample['position_error_m'] = float(np.linalg.norm(
                fk(sample['sdk_positions_rad'])[:3,3] - fk(sample['accepted_positions_rad'])[:3,3]))
    except ValueError as exc:
        return {'hypothesis': 'all_axes_repeatable_fixed_bias' if responsive else 'no_axis_response',
                'results': results, 'terminal': str(exc),
                'steps_issued_in_simulation': correction.steps}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',required=True)
    parser.add_argument('--parameters',required=True)
    parser.add_argument('--at',type=float,required=True)
    parser.add_argument('--output',required=True)
    args = parser.parse_args()
    if args.bag.endswith('.active') or not math.isfinite(args.at) or args.at <= 0:
        raise ValueError('closed bag and finite historical time required')
    with open(args.parameters) as f:
        params=yaml.safe_load(f)
    fk=SerialUrdfFk(params['robot_description'],ARM_NAMES)
    sdk,accepted,epochs=[],[],[]
    topics=['/alicia_d/sdk_command','/alicia_d/accepted_joint_states','/alicia_d/control_reference_epoch']
    with rosbag.Bag(args.bag,'r') as bag:
        for topic,msg,stamp in bag.read_messages(topics=topics):
            if stamp.to_sec()>args.at: continue
            if topic==topics[2]:
                if msg.frame_id!='sdk_reference_epoch': raise ValueError('invalid epoch')
                epochs.append(msg.stamp.to_nsec())
            elif stamp.to_sec()>=args.at-1.:
                (sdk if topic==topics[0] else accepted).append(msg)
    initial=stationary_following_error(fk,sdk,accepted,now_sec=args.at,
                                      epoch_ns=epochs[-1] if epochs else None)
    report={'mode':'offline_measured_initial_state_plus_hypotheses_only',
            'bag':args.bag,'parameters':args.parameters,'measured_initial':initial,
            'simulated_responsive':replay(fk,initial),
            'simulated_stalled':replay(fk,initial,False),
            'motion_executed':False,'real_convergence_proven':False}
    with open(args.output,'x') as f: json.dump(report,f,indent=2,allow_nan=False)
    print(json.dumps(report,indent=2,allow_nan=False))


if __name__=='__main__': main()
