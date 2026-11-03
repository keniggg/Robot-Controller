#!/usr/bin/env python3
"""Offline adversarial callback schedule; no ROS node/service/action is used."""
from copy import deepcopy
import argparse
import hashlib
import json
from pathlib import Path
import sys

WORK = Path(__file__).resolve().parents[1]
STAGE = WORK / '.runtime/initial_precommit_staging_20261009'
sys.path.insert(0, str(WORK / 'src/alicia_flexible_grasp_supervisor/tests'))
sys.path.insert(0, str(STAGE / 'src/alicia_grasp_modes/src'))
sys.path.insert(0, str(STAGE / 'src/alicia_grasp_modes/tests'))
from _pytest.monkeypatch import MonkeyPatch
from std_msgs.msg import Bool
from test_initial_static_identity_completion import gap
from test_initial_preflight_task import prepared


def run_case(kind):
    with MonkeyPatch().context() as patch:
        source = gap.__wrapped__(patch)
        case = prepared.__wrapped__(source, patch)
        node = case.node
        calls = []

        def ownership_change():
            if kind.startswith('manual'):
                case.params['/gui/joint_direct_mode'] = True
                node._initial_preflight_manual(Bool(data=True))
                case.params['/gui/joint_direct_mode'] = False
                node._initial_preflight_manual(Bool(data=False))
            else:
                original = deepcopy(node._following_accepted_history[-1])
                moved = deepcopy(original)
                moved.position = list(moved.position)
                moved.position[0] += .006
                node.accepted_joint_cb(moved)
                node.accepted_joint_cb(original)
        def acknowledgement(action):
            assert action == 'commit'
            if kind.endswith('during_ack'):
                ownership_change()
            return dict(state='committed', accepted=True)

        patch.setattr(node, '_initial_preflight_rpc', acknowledgement)
        error = None
        try:
            with node._grasp6d_plan_guard():
                node._initial_preflight_commit()
                if kind.endswith('after_ack'):
                    ownership_change()
                # Strict wrapper is an inert counter, not a service proxy.
                wrapped = node._initial_preflight_wrap_strict(lambda *a, **k: calls.append('inert_call'))
                wrapped()
                if kind == 'later_checked_submission_delegates':
                    ownership_change()
                    # Only check closure retirement: caller's original later
                    # plan/action validation remains outside this inert seam.
                    wrapped()
        except Exception as exc:
            error = str(exc)
        return dict(case=kind, disqualifications=node._initial_preflight_attempt['disqualifications'],
            hard_revoked=bool(node._execution_authority_revoked),
            token_state=node._initial_preflight_token['state'], inert_calls=calls,
            error=error, limitation='ACK transport stub; real manual/joint callbacks and real commit/strict wrapper; no physical action')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    path = STAGE / 'src/alicia_grasp_modes/src/alicia_grasp_modes/initial_preflight_task.py'
    result = dict(task_source=str(path), task_source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        cases=[run_case(k) for k in ('manual_during_ack', 'pose_during_ack',
            'manual_after_ack', 'pose_after_ack', 'later_checked_submission_delegates')])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True)+'\n')
    print(json.dumps(result, indent=2, sort_keys=True))
