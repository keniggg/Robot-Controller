#!/usr/bin/env python3
"""Compare frozen far/near RGB-D windows offline; never access live ROS.

ROI variants are sensitivity experiments, not replacement execution geometry.
The output deliberately retains the production baseline and rejected results.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag
import yaml

from replay_grasp_registration_bag import (
    read_sources, rebuild_fused_view, plain, surface, TargetTrackIdentity,
)


def comparison(first, second, config):
    angle = np.degrees(np.arccos(np.clip(np.dot(
        first.support_normal_base, second.support_normal_base), -1., 1.)))
    return {
        'normal_angle_deg': float(angle),
        'local_plane_separation_m': surface.support_plane_separation_m(first, second),
        'registration': plain(surface.register_surface_view(first, second, config)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--parameters', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    params = yaml.safe_load(Path(args.parameters).read_text())
    params['remote'] = params['grasp_6d']['remote']
    config = surface.RegistrationConfig(**params['remote'].get('multiview', {}))
    windows = {}
    plans = []
    reached = None
    target_epoch = None
    with rosbag.Bag(args.bag) as bag:
        for topic, msg, _time in bag.read_messages(topics=[
            '/grasp_6d/gate_audit', '/grasp/near_field_phase',
            '/grasp_6d/preview_plan_enriched', '/rosout_agg',
        ]):
            if topic.endswith('/gate_audit'):
                audit = json.loads(msg.data)
                funnel = audit.get('summary', {}).get('pipeline_funnel', {})
                evidence = funnel.get('snapshot_evidence', {})
                if audit.get('selected') and not evidence.get('near_field', False):
                    windows['far'] = evidence['source_stamp_ns']
            elif topic == '/grasp/near_field_phase' and msg.active:
                reached = msg.reference_source_stamp.to_nsec()
            elif topic.endswith('/preview_plan_enriched') and msg.valid:
                plans.append(msg)
            elif topic == '/rosout_agg' and 'pipeline metrics: ' in msg.msg:
                metrics = json.loads(msg.msg.split('pipeline metrics: ', 1)[1])
                evidence = metrics.get('snapshot_evidence', {})
                if evidence.get('near_field'):
                    windows['near'] = evidence['source_stamp_ns']
                    target_epoch = metrics.get('target_epoch')
    if not windows.get('far') or not windows.get('near') or not reached:
        raise ValueError('missing committed far/near source window or reached-view handoff')
    windows['reached'] = [reached]
    if not plans or type(target_epoch) is not int or target_epoch <= 0:
        raise ValueError('missing recorded target identity epoch')
    if len({p.target_track_id for p in plans}) != 1:
        raise ValueError('source plans do not belong to one target track')
    identity = TargetTrackIdentity(target_epoch, plans[-1].target_track_id)
    stamps = sorted({s for values in windows.values() for s in values})
    frames, joints, buffer, files = read_sources(args.bag, stamps)
    production_ratio = params['remote'].get('support_bbox_expand_ratio', .30)
    output = {
        'read_only': True, 'execution_authority': False,
        'parameters_sha256': hashlib.sha256(Path(args.parameters).read_bytes()).hexdigest(),
        'bag_files': [str(p) for p in files], 'source_windows': windows,
        'target_epoch': target_epoch, 'target_track_id': identity.track_id,
        'production_expand_ratio': production_ratio, 'experiments': [],
        'wire_plans': [{
            'plan_id': p.plan_id, 'stamp_ns': p.header.stamp.to_nsec(),
            'normal': [getattr(p.object_geometry.support_normal_base, a) for a in 'xyz'],
            'offset_m': p.object_geometry.support_offset_m,
            'refinement_status': p.refinement_status,
        } for p in plans],
    }
    # Fixed, preregistered sensitivity set; never select whichever passes.
    for ratio in dict.fromkeys([production_ratio, .15, .60, 1.0]):
        experiment = {'support_bbox_expand_ratio': ratio, 'views': {}, 'comparisons': {}}
        views = {}
        variant = deepcopy(params)
        variant['remote']['support_bbox_expand_ratio'] = ratio
        for name, window in windows.items():
            try:
                view, detail = rebuild_fused_view(window, frames, joints, buffer, variant, identity)
                views[name] = view
                experiment['views'][name] = {
                    'support_normal_base': view.support_normal_base.tolist(),
                    'support_offset_m': view.support_offset_m,
                    'point_count': len(view.points_base), 'detail': detail,
                }
            except ValueError as exc:
                experiment['views'][name] = {'error': str(exc)}
        for first, second in [('far', 'near'), ('far', 'reached'), ('reached', 'near')]:
            if first in views and second in views:
                experiment['comparisons'][first + '_to_' + second] = comparison(
                    views[first], views[second], config)
        output['experiments'].append(experiment)
        print(json.dumps(experiment, allow_nan=False), flush=True)
    with Path(args.output).open('x') as handle:
        json.dump(output, handle, indent=2, allow_nan=False)


if __name__ == '__main__':
    main()
