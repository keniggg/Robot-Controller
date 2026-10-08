#!/usr/bin/env python3
"""Offline archived contact geometry diagnostic; never starts ROS or moves hardware."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('fixture',nargs='?',type=Path,
        default=ROOT/'tests/fixtures/contact_height_margin_20260927.json')
    parser.add_argument('--translation-evidence-limit-m',type=float)
    args=parser.parse_args()
    record=json.loads(args.fixture.read_text())
    geometry=dict(record['geometry'],gripper=GripperGeometry(.05,.002,
        np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003))
    limit=(record['translation_evidence_limit_m'] if args.translation_evidence_limit_m is None
           else args.translation_evidence_limit_m)
    result=evaluate_contact_height_margin(translation_evidence_limit_m=limit,**geometry)
    result.update(source_record_id=record['source_record_id'],plan_id=record['plan_id'],
                  counterfactual_sensitivity=record['counterfactual_sensitivity'])
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0 if result['evaluated'] else 2


if __name__=='__main__':
    sys.exit(main())
