#!/usr/bin/env python3
"""Offline replay of generation 3's initial preflight handoff (no ROS I/O)."""
from copy import deepcopy
import argparse
import hashlib
import json
from pathlib import Path
import sys

import pytest
from _pytest.monkeypatch import MonkeyPatch
import rosbag
import rospy

WORK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK / 'src/alicia_grasp_modes/tests'))
from test_task_modes import configured, module, base
from alicia_grasp_modes.unknown_pending_identity import (
    initial_station_template, reference_binding, validate_support_receipt,
)
from alicia_grasp_modes.tabletop import Config


def run(bag_path):
    ref_ns = 1791594201510251998
    gap_ns = 1791594201710199594
    recovery_ns = 1791594201808995485
    status = {}
    geometry = None
    plan = None
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, msg, received in bag.read_messages(topics=[
                '/perception/unknown/detector_status', '/grasp_6d/object_geometry',
                '/grasp_6d/plan_enriched']):
            if topic.endswith('detector_status'):
                parsed = json.loads(msg.data)
                ns = parsed.get('source_stamp_ns')
                if ns in (ref_ns, gap_ns, recovery_ns):
                    status[ns] = (parsed, received.to_nsec())
            elif topic.endswith('object_geometry'):
                if msg.header.stamp.to_nsec() == gap_ns:
                    geometry = (deepcopy(msg), received.to_nsec())
            elif msg.valid:
                plan = deepcopy(msg)
    assert plan is not None and geometry is not None
    reference_status, _ = status[ref_ns]
    gap_status, gap_received = status[gap_ns]
    recovery_status, recovery_received = status[recovery_ns]
    selection = dict(mode='unknown', strategy='observe_return', generation=3,
                     stamp_ns=reference_status['selection_stamp_ns'])
    defaults = Config()
    reference = dict(source_ns=ref_ns, selection=selection,
        binding=reference_binding(reference_status, selection), maximum_sec=.8,
        deadline_ns=ref_ns + 800000000,
        identity_template=initial_station_template(reference_status, selection),
        initial_station_measurement=reference_status['initial_station_measurement'],
        support_policy={key: getattr(defaults, key) for key in (
            'support_max_normal_change_deg', 'support_max_local_shift_m',
            'support_threshold_m', 'support_min_ratio')},
        footprint_vertices=len(reference_status['reference_support_footprint_base']))
    source_id = validate_support_receipt(gap_status, reference, gap_received)
    reference['source_id'] = source_id
    reference['identity_completion'] = deepcopy(
        gap_status['source_qualification']['identity_completion'])
    assert validate_support_receipt(recovery_status, reference, recovery_received,
                                    recovery=True) == source_id

    with MonkeyPatch().context() as patch:
        node, params = configured.__wrapped__(patch)
        params[module.SELECTION_PARAM] = dict(selection, stamp_ns=str(selection['stamp_ns']))
        assert node.reset_mode_cb(None).success
        node._mode_bound_selection = deepcopy(node._mode_selection)
        node.active = True
        node.stage = 3
        node._execution_authority_revoked = False
        node._bound_execution_plan = deepcopy(plan)
        node._bound_execution_plan_id = plan.plan_id
        node._bound_execution_plan_digest = node._execution_plan_digest(plan)
        patch.setattr(rospy.Time, 'now', staticmethod(lambda: rospy.Time(
            geometry[1] // 1000000000, geometry[1] % 1000000000)))
        before = node._execution_authority_revoked
        node.target_geometry_cb(geometry[0])
        after = node._execution_authority_revoked
        assert before is False and after is True
        called = []
        validation, response = node._invoke_plan_bound_action(
            plan, {}, 'gripper open after observation preflight',
            lambda: called.append('physical action'))
        assert validation.code == 'EXECUTION_AUTHORITY_REVOKED'
        assert response is None and called == []
        # A later valid geometry callback cannot resurrect revoked authority.
        recovered_geometry = deepcopy(plan.object_geometry)
        recovered_geometry.header.stamp = rospy.Time(
            recovery_ns // 1000000000, recovery_ns % 1000000000)
        recovered_geometry.valid = True
        node.target_geometry_cb(recovered_geometry)
        assert node._execution_authority_revoked
    return dict(schema_version=1, mode='offline_actual_bag_callback_replay',
        bag=str(bag_path), plan_id=plan.plan_id, original_plan_track=plan.target_track_id,
        invalid_geometry_track=geometry[0].target_track_id,
        invalid_geometry_reason=geometry[0].failure_reason,
        reference_source_ns=ref_ns, pending_source_ns=gap_ns,
        pending_received_ns=gap_received, invalid_geometry_received_ns=geometry[1],
        recovery_source_ns=recovery_ns, recovery_received_ns=recovery_received,
        identity_deadline_ns=reference['identity_completion']['identity_completion_deadline_ns'],
        pending_support_receipt_valid=True, recovered_support_receipt_valid=True,
        callback_revoked_frozen_plan=True, later_geometry_cannot_unrevoke=True,
        action_validation_code=validation.code, physical_action_calls=called,
        limitations=['Replays recorded task geometry callback and receipt validators; '
                     'does not infer the exact cross-process callback scheduling.',
                     'Recovery geometry negative test derives from recorded plan; '
                     'actual producer recovery support receipt is replayed unchanged.'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = run(args.bag)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(result, indent=2, sort_keys=True) + '\n').encode()
    args.output.write_bytes(payload)
    print(json.dumps(dict(output=str(args.output), sha256=hashlib.sha256(payload).hexdigest(),
                         replay_passed=True, physical_action_calls=0)))
