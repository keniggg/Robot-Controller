#!/usr/bin/env python3
import argparse
import json
import sys

from alicia_flexible_grasp.vision.remote_grasp6d_client import RemoteGrasp6DClient


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Check Alicia remote GraspNet baseline HTTP service')
    parser.add_argument('url', help='Remote service URL, for example http://192.168.26.1:8000')
    parser.add_argument('--timeout', type=float, default=3.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        health = RemoteGrasp6DClient(args.url, timeout_sec=args.timeout).health()
    except Exception as exc:
        print('FAIL: %s' % exc, file=sys.stderr)
        return 1

    print(json.dumps(health, indent=2, ensure_ascii=False))
    if not bool(health.get('ok', False)):
        print('FAIL: remote service is reachable but reports unhealthy', file=sys.stderr)
        return 2
    print('OK: remote 6D grasp service is reachable')
    return 0


if __name__ == '__main__':
    sys.exit(main())
