#!/usr/bin/env python3
"""Read a CLOSED bag and its frozen parameters; no ROS node/control I/O.

Report a stationary SDK/encoder FK gap and exact six-joint Shapley vector
contributions. Counterfactual FK substitutions are diagnostic only: they are
NOT safe compensation commands or evidence of accurate physical calibration.
"""
import argparse
import json
import math

import numpy as np
import rosbag
import yaml

from alicia_flexible_grasp.robot.stationary_following import (
    ARM_NAMES, SerialUrdfFk, stationary_following_error,
)


def displacement_attribution(fk, sdk, accepted):
    sdk, accepted = np.asarray(sdk, dtype=float), np.asarray(accepted, dtype=float)
    if sdk.shape != (6,) or accepted.shape != (6,) or tuple(fk.names) != ARM_NAMES:
        raise ValueError('attribution requires exactly the six ordered arm joints')
    positions = {}
    for mask in range(64):
        q = np.array([accepted[j] if mask & (1 << j) else sdk[j] for j in range(6)])
        positions[mask] = fk(q)[:3, 3]
    contributions = np.zeros((6, 3))
    for joint in range(6):
        for mask in range(64):
            if mask & (1 << joint):
                continue
            size = bin(mask).count('1')
            weight = math.factorial(size)*math.factorial(5-size)/math.factorial(6)
            contributions[joint] += weight*(positions[mask | (1 << joint)]-positions[mask])
    error = positions[63]-positions[0]
    magnitude = float(np.linalg.norm(error))
    return {
        'method': 'exact six-joint Shapley FK displacement, not commanded compensation',
        'joint_displacement_contributions_mm': dict(zip(ARM_NAMES, (contributions*1000).tolist())),
        'projection_along_total_error_mm': (dict(zip(
            ARM_NAMES, (contributions @ (error/magnitude)*1000).tolist()))
            if magnitude > 0 else None),
        'sum_vector_mm': (contributions.sum(axis=0)*1000).tolist(),
        'total_vector_mm': (error*1000).tolist(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--parameters', required=True)
    parser.add_argument('--at', required=True, type=float,
                        help='historical ROS seconds at the end of a stationary window')
    args = parser.parse_args()
    if not math.isfinite(args.at) or args.at <= 0 or args.bag.endswith('.active'):
        raise ValueError('require a closed bag and positive finite historical time')
    with open(args.parameters) as stream:
        parameters = yaml.safe_load(stream)
    fk = SerialUrdfFk(parameters['robot_description'], ARM_NAMES)
    sdk, accepted, epochs = [], [], []
    topics = ['/alicia_d/sdk_command', '/alicia_d/accepted_joint_states',
              '/alicia_d/control_reference_epoch']
    with rosbag.Bag(args.bag, 'r') as bag:
        for topic, msg, stamp in bag.read_messages(topics=topics):
            if stamp.to_sec() > args.at:
                continue
            if topic == topics[2]:
                if msg.frame_id != 'sdk_reference_epoch':
                    raise ValueError('invalid recorded reference epoch frame')
                epochs.append(msg.stamp.to_nsec())
            elif stamp.to_sec() >= args.at-1.0:
                (sdk if topic == topics[0] else accepted).append(msg)
    report = stationary_following_error(fk, sdk, accepted, now_sec=args.at,
                                       epoch_ns=epochs[-1] if epochs else None)
    print(json.dumps({
        'read_only': True, 'bag': args.bag, 'parameters': args.parameters,
        'following_evidence': report,
        'attribution': displacement_attribution(
            fk, report['sdk_positions_rad'], report['accepted_positions_rad']),
    }, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
