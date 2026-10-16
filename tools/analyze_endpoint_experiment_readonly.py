#!/usr/bin/env python3
"""Replay one completed endpoint diagnostic from a CLOSED bag, without ROS I/O.

Separates fixed original goal error, current wire/encoder gap, raw encoder
evidence and host write success. No physical calibration or grasp certificate.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np
import rosbag
import yaml

from alicia_flexible_grasp.robot.endpoint_correction import sdk_counts
from alicia_flexible_grasp.robot.stationary_following import (
    ARM_NAMES, SDK_QUANTUM_RAD, SerialUrdfFk, stationary_following_error,
)

SDK = '/alicia_d/sdk_command'
ACCEPTED = '/alicia_d/accepted_joint_states'
EPOCH = '/alicia_d/control_reference_epoch'
RAW = '/alicia_d/sdk_diagnostics'
GOAL = '/alicia_controller/follow_joint_trajectory/goal'
RESULT = '/alicia_controller/follow_joint_trajectory/result'
GUI = '/gui/joint_direct_mode'
TASK = '/grasp/state'


def counts(message):
    named = dict(zip(message.name, message.position))
    return sdk_counts([named[name] for name in ARM_NAMES]).tolist()


def raw_joint_row(message):
    for status in message.status:
        values = {entry.key: entry.value for entry in status.values}
        if (status.name == 'alicia_d/sdk_raw_frame'
                and bytes.fromhex(values.get('command_function_hex', '')) == b'\x06\x00'):
            payload = bytes.fromhex(values['payload_hex'])
            if len(payload) < 15:
                return {'invalid': 'short_joint_payload'}
            words = [int.from_bytes(payload[i:i+2], 'little') for i in range(0, 12, 2)]
            valid = (max(words) <= 4095 and words != [0]*6 and words != [4095]*6)
            return {'counts': words, 'status': '%02x' % payload[14],
                    'valid_encoder_layout': valid, 'payload_hex': payload.hex(),
                    'host_parse_stamp_ns': message.header.stamp.to_nsec()}
    return None


def summarize_rows(rows):
    if not rows:
        return {'sample_count': 0}
    values = np.asarray([row[1] for row in rows], dtype=int)
    return {'sample_count': len(rows), 'first_sec': rows[0][0], 'last_sec': rows[-1][0],
            'first_counts': rows[0][1], 'last_counts': rows[-1][1],
            'minimum_counts': values.min(axis=0).tolist(),
            'maximum_counts': values.max(axis=0).tolist()}


def analyze(bag_path, parameters_path, execution):
    if str(bag_path).endswith('.active') or not execution['execution_requested']:
        raise ValueError('closed bag and completed executed diagnostic required')
    start, completed = execution['started_sec'], execution['completed_sec']
    if not all(math.isfinite(t) for t in (start, completed)) or completed <= start:
        raise ValueError('invalid experiment times')
    terminal = json.loads(execution['message'])
    with open(parameters_path) as stream:
        fk = SerialUrdfFk(yaml.safe_load(stream)['robot_description'], ARM_NAMES)
    topics = [SDK, ACCEPTED, EPOCH, RAW, GOAL, RESULT, GUI, TASK]
    messages = {topic: [] for topic in topics}
    with rosbag.Bag(str(bag_path), 'r') as bag:
        for topic, message, receipt in bag.read_messages(topics=topics):
            messages[topic].append((receipt.to_sec(), message))
    last = min(messages[SDK][-1][0], messages[ACCEPTED][-1][0]) - .05
    if last < completed + 1.:
        raise ValueError('missing post-command hold evidence')
    goal_counts = terminal['fixed_goal_counts']
    fixed_q = (np.asarray(goal_counts)-2048)*SDK_QUANTUM_RAD
    fixed_pose = fk(fixed_q)
    windows = []
    for label, at in [('before', start), ('after_1_sec', completed+1.),
                      ('after_10_sec', completed+10.), ('late_hold', last)]:
        if at > last:
            continue
        epochs = [m.stamp.to_nsec() for t, m in messages[EPOCH] if t <= at]
        if not epochs or epochs[-1] != terminal['epoch_ns']:
            raise ValueError('missing or changed recorded epoch')
        report = stationary_following_error(fk,
            [m for t, m in messages[SDK] if at-1. <= t <= at],
            [m for t, m in messages[ACCEPTED] if at-1. <= t <= at],
            now_sec=at, epoch_ns=epochs[-1])
        if label == 'before' and sdk_counts(report['sdk_positions_rad']).tolist() != goal_counts:
            raise ValueError('original SDK goal differs from recorded baseline')
        measured = fk(report['accepted_positions_rad'])
        report.update(window=label, fixed_goal_counts=goal_counts,
            fixed_goal_position_error_m=float(np.linalg.norm(measured[:3, 3]-fixed_pose[:3, 3])),
            fixed_goal_orientation_error_rad=math.acos(float(np.clip(
                (np.trace(fixed_pose[:3, :3].T @ measured[:3, :3])-1)/2, -1., 1.))))
        windows.append(report)
    raw = [(t, raw_joint_row(m)) for t, m in messages[RAW]]
    raw = [(t, row) for t, row in raw if row is not None]
    phases = {}
    for label, low, high in [('before', start-1., start),
                             ('execution', start, completed),
                             ('post_hold', completed, last)]:
        selected = [(t, row) for t, row in raw if low <= t <= high]
        raw_counts = [row['counts'] for _, row in selected if row.get('valid_encoder_layout')]
        accepted_counts = [counts(m) for t, m in messages[ACCEPTED] if low <= t <= high]
        phases[label] = {
            'sdk': summarize_rows([(t, counts(m)) for t, m in messages[SDK] if low <= t <= high]),
            'accepted': summarize_rows([(t, counts(m)) for t, m in messages[ACCEPTED] if low <= t <= high]),
            'raw': summarize_rows([(t, row['counts']) for t, row in selected
                                   if row.get('valid_encoder_layout')]),
            'raw_invalid_encoder_layout_count': sum(not row.get('valid_encoder_layout') for _, row in selected),
            'raw_status_histogram': dict(Counter(row.get('status', 'unknown') for _, row in selected)),
            # Same-order sequences, not a claim of synchronized MCU timestamps.
            'raw_accepted_count_sequences_match': bool(raw_counts and raw_counts == accepted_counts),
        }
    before, after = windows[0], windows[-1]
    return {'mode': 'closed_bag_actual_endpoint_response_not_hypothetical',
        'bag': str(bag_path), 'parameters': str(parameters_path),
        'executed_steps': terminal['steps_completed'], 'original_terminal': terminal,
        'model_sha256': fk.model_sha256, 'windows': windows, 'phases': phases,
        'sdk_delta_counts': (sdk_counts(after['sdk_positions_rad'])-sdk_counts(before['sdk_positions_rad'])).tolist(),
        'measured_delta_counts': (sdk_counts(after['accepted_positions_rad'])-sdk_counts(before['accepted_positions_rad'])).tolist(),
        'post_command_observation_sec': last-completed,
        'action_goals': [{'receipt_sec': t, 'goal_id': m.goal_id.id,
                          'duration_sec': m.goal.trajectory.points[-1].time_from_start.to_sec()}
                         for t, m in messages[GOAL] if start <= t <= last],
        'action_results': [{'receipt_sec': t, 'status': m.status.status,
                            'error_code': m.result.error_code, 'error_string': m.result.error_string}
                           for t, m in messages[RESULT] if start <= t <= last],
        'gui_events': [{'receipt_sec': t, 'direct': m.data} for t, m in messages[GUI]],
        'task_events': [{'receipt_sec': t, 'active': m.active, 'success': m.success}
                        for t, m in messages[TASK]],
        'successful_host_write_is_device_ack': False,
        'grasp_or_physical_calibration_success': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for field in ('bag', 'parameters', 'execution', 'output'):
        parser.add_argument('--'+field, required=True)
    args = parser.parse_args()
    with open(args.execution) as stream:
        execution = json.load(stream)
    report = analyze(args.bag, args.parameters, execution)
    with Path(args.output).open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(json.dumps({key: report[key] for key in (
        'sdk_delta_counts', 'measured_delta_counts', 'post_command_observation_sec',
        'action_goals', 'action_results', 'phases')}, indent=2, allow_nan=False))
    for window in report['windows']:
        print(window['window'], 'fixed_goal_mm=', window['fixed_goal_position_error_m']*1000,
              'sdk_following_mm=', window['position_error_m']*1000)


if __name__ == '__main__':
    main()
