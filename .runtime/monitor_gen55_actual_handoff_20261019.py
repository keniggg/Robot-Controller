"""File-only GEN55 adapter for logs spanning the accepted cold restart.

Both input logs are checked against exact known paths before delegating to the
existing observer. --prepare records metadata only; --record needs a new
MODE_ATTEMPT. This script never starts, stops, retries, or authorizes a task.
"""
import argparse
import hashlib
import sys
from pathlib import Path

import monitor_gen52_actual_handoff_20261018 as observer

W = Path(__file__).resolve().parents[1]
ORIGINAL = observer.original
CURRENT_IDENTITY = ORIGINAL.E/'ros_log/full_validated_gen54_restart_20261020T012700Z/gen54_current_runtime_identity.json'
CURRENT_IDENTITY_SHA256 = '7e256ee6f5cb64756d40e4c2b07468a791d10704c81d4154144b141a11f23bd3'
CONSUMERS = ORIGINAL.N/'gen55_runtime/compute_gateway_deployment_r1/consumers.log'
GATEWAY = CURRENT_IDENTITY.parent/'core_ros/12936474-cc26-11f1-8561-6b34442c9236/motion_gateway-7.log'
OUTPUT = ORIGINAL.V/'gen55_actual_handoff_monitor_r1'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare', action='store_true')
    parser.add_argument('--record', type=Path)
    parser.add_argument('--consumers', type=Path, required=True)
    parser.add_argument('--gateway-log', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert hashlib.sha256(CURRENT_IDENTITY.read_bytes()).hexdigest() == CURRENT_IDENTITY_SHA256
    assert args.consumers.resolve() == CONSUMERS.resolve()
    assert args.gateway_log.resolve() == GATEWAY.resolve()
    assert args.output.resolve() == OUTPUT.resolve()
    if args.record is not None:
        assert not args.prepare
        assert ORIGINAL.inside(args.record, ORIGINAL.E/'grasp_records')
    else:
        assert args.prepare
    # The existing observer permits one experiment log root. The exact-path
    # checks above retain a narrower input set while its parser spans both
    # historical and cold-restart roots. Its output root stays unchanged.
    old_root = ORIGINAL.N
    try:
        ORIGINAL.N = ORIGINAL.E/'ros_log'
        observer.main()
    finally:
        ORIGINAL.N = old_root


if __name__ == '__main__':
    main()
