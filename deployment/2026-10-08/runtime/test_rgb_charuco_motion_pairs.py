"""Relative rotation and metric displacement diagnostics; pure offline maths."""
import importlib.util
from pathlib import Path
import unittest

import cv2
import numpy as np

PATH=Path(__file__).parent/'gen4_root_cause/analyze_rgb_charuco_motion_pairs.py'
SPEC=importlib.util.spec_from_file_location('rgb_motion_companion',PATH)
M=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(M)


def row(label,B,C):return dict(label=label,T_base_optical=C.tolist(),
   rgb={'T_optical_board':B.tolist(),'object_points_m':[[0,0,0],[.1,0,0],[0,.1,0]]},
   binding={'source_publication_header_ns':1,'q_bracket_before':[1,[0]*7]})


class MotionMetricTests(unittest.TestCase):
 def test_axis_projection_recovers_known_camera_rotation(self):
  C0=np.eye(4);C1=np.eye(4);C1[:3,:3]=cv2.Rodrigues(np.array([-.04,0,0]))[0]
  B0=np.eye(4);B0[2,3]=.4;B1=np.linalg.inv(C1)@B0
  result=M.motion_metrics(row('a',B0,C0),row('b',B1,C1))
  for name in ['RGB','recorded_TF']:
   metric=result['rotation_measurements'][name]
   self.assertAlmostEqual(metric['rotation_angle_deg'],np.degrees(.04),places=8)
   self.assertAlmostEqual(metric['rotation_projected_on_TF_axis_deg'],np.degrees(.04),places=8)
   self.assertLess(metric['nonparallel_rotation_deg'],1e-8)

 def test_stationary_pair_does_not_invent_rotation_axis(self):
  B=np.eye(4);B[2,3]=.4;C=np.eye(4)
  result=M.motion_metrics(row('a',B,C),row('b',B,C))
  self.assertIn('TF_axis_projection_unavailable',result['rotation_measurements']['RGB'])
  self.assertNotIn('rotation_projected_on_TF_axis_deg',result['rotation_measurements']['RGB'])


if __name__=='__main__':unittest.main()
