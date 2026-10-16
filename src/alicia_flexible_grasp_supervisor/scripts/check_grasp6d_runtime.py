#!/usr/bin/env python3
import argparse
import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.grasp6d_adapter import default_grasp6d_root, inspect_grasp6d_runtime


def main():
    parser = argparse.ArgumentParser(description='Check Alicia-D 6D grasp runtime dependencies.')
    parser.add_argument('--root', default=str(default_grasp6d_root()), help='Path to src/real-arm/alicia_d_grasp_6d')
    parser.add_argument('--checkpoint-path', default='', help='Path to GraspNet checkpoint tar file')
    args = parser.parse_args()

    report = inspect_grasp6d_runtime(root=args.root, checkpoint_path=args.checkpoint_path)
    print('ready:', report.ready)
    print('root:', pathlib.Path(args.root).expanduser())
    print('checkpoint_path:', args.checkpoint_path or '<unset>')
    print('versions:')
    for key in sorted(report.versions):
        print('  %s: %s' % (key, report.versions[key]))
    if report.missing:
        print('missing:')
        for item in report.missing:
            print('  - %s' % item)
    else:
        print('missing: none')
    return 0 if report.ready else 1


if __name__ == '__main__':
    raise SystemExit(main())
