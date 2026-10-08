"""Independent board maths regressions; no ROS imports or device calls."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np
import cv2
import pyrealsense2 as rs

PATH=Path(__file__).parent/'gen4_root_cause/analyze_rgb_charuco_coordinate_chain.py'
SPEC=importlib.util.spec_from_file_location('independent_RGB_board',PATH)
M=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(M)


def camera_parameters():
 color=dict(width=640,height=480,fx=395.7682800292969,fy=395.6027526855469,
    cx=317.9961242675781,cy=244.833984375,model='inverse_brown_conrady',
    coeffs=[-.05372992530465126,.05834611505270004,.002122000325471163,
            .0007625872385688126,-.019704923033714294])
 return dict(width=640,height=480,frame_id='camera_link',
    color_topic='/supervisor/camera/color/image_raw',runtime_profile=dict(
     schema_version=1,source='realsense_active_profile',sdk_version='2.56.5',
     device={'serial_number':'synthetic_no_device'},color_intrinsics=color,
     color_projection_correction={'enabled':False}))


def board_spec():return dict(board_size=[12,9],square_length_m=.015,
   marker_length_m=.01125,dictionary_id='DICT_5X5_100',legacy_pattern=True,detection_scale=1.5)


class IndependentRGBTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.model=M.runtime_model(camera_parameters(),rs)
  cls.points=np.asarray([[.015*x,.015*y,0]for y in range(1,9)for x in range(1,12)])
  cls.ids=np.arange(len(cls.points));cls.rvec=np.array([.23,-.12,.04]);cls.tvec=np.array([-.075,-.05,.4])
  cls.pixels=cls.model.project(cls.points,cls.rvec,cls.tvec)

 def detection(self,pixels=None,points=None):
  points=self.points if points is None else points
  return dict(ids=self.ids,pixels=self.pixels.copy()if pixels is None else pixels,
      object_points=points,unit_board_xyz=self.points/.015,marker_count=40)

 def test_known_RGB_pose_recovers_with_inverse_brown_rays(self):
  result=M.solve_rgb(self.model,self.points,self.pixels);T=np.array(result['T_optical_board'])
  np.testing.assert_allclose(T[:3,:3],cv2.Rodrigues(self.rvec)[0],atol=2e-6)
  np.testing.assert_allclose(T[:3,3],self.tvec,atol=2e-7)
  self.assertLess(result['RGB_reprojection_px']['rms'],.0001)
  self.assertGreaterEqual(len(result['candidates']),2)

 def test_RGB_translation_scale_is_not_hidden_or_depth_calibrated(self):
  original=M.solve_rgb(self.model,self.points,self.pixels)
  other=M.solve_rgb(self.model,self.points*1.2,self.pixels)
  A,B=np.array(original['T_optical_board']),np.array(other['T_optical_board'])
  np.testing.assert_allclose(B[:3,:3],A[:3,:3],atol=2e-6)
  np.testing.assert_allclose(B[:3,3],A[:3,3]*1.2,atol=3e-7)

 def test_heldout_RGB_pixels_cannot_choose_training_pose(self):
  original=M.rgb_review(self.model,self.detection())
  fold=M.board_spatial_folds(self.points/.015);pixels=self.pixels.copy();pixels[fold==0]+=[8,-4]
  other=M.rgb_review(self.model,self.detection(pixels))
  a=original['spatial_folds'][0];b=other['spatial_folds'][0]
  np.testing.assert_array_equal(a['T_optical_board'],b['T_optical_board'])
  self.assertGreater(b['heldout_RGB_reprojection_px']['rms'],8)

 def test_planar_spatial_tiles_remain_in_single_fold(self):
  unit=np.array([[1.1,1.2,0],[1.8,1.7,0],[2.1,1.1,0],[2.8,1.8,0],[4.1,1.1,0],[4.7,1.8,0]])
  f=M.board_spatial_folds(unit);self.assertEqual(f[0],f[1]);self.assertEqual(f[2],f[3]);self.assertEqual(f[4],f[5])

 def test_missing_depth_remains_missing_and_RGB_result_unchanged(self):
  depth=np.zeros((480,640));patch=M.patch_depth(depth,self.pixels)
  self.assertTrue(all(z is None for z in patch['optical_z_m']))
  before=M.rgb_review(self.model,self.detection())
  with self.assertRaisesRegex(ValueError,'DEPTH_BOARD_INTERIOR_INSUFFICIENT'):
   M.depth_review(self.model,self.detection(),depth,before)
  after=M.rgb_review(self.model,self.detection())
  self.assertEqual(before['T_optical_board'],after['T_optical_board'])

 def test_same_pixel_depth_bias_is_reported_without_fixing_RGB(self):
  pixels=self.model.project(self.points,np.zeros(3),[-.09,-.06,.4])
  detection=self.detection(pixels);rgb=M.rgb_review(self.model,detection)
  depth=np.full((480,640),.410)
  result=M.depth_review(self.model,detection,depth,rgb)
  self.assertAlmostEqual(result['depth_minus_RGB_plane_z_mm']['median'],10,places=3)
  T=np.array(rgb['T_optical_board']);self.assertAlmostEqual(T[2,3],.4,places=6)
  self.assertGreater(result['depth_supported_grid']['similarity_square_mm'],15.3)

 def test_static_board_relative_transform_sign_and_TF_disagreement(self):
  B0=np.eye(4);B0[:3,3]=[-.08,-.05,.4]
  C0=np.eye(4);C1=np.eye(4);C1[:3,:3]=cv2.Rodrigues(np.array([.03,0,0]))[0];C1[:3,3]=[.01,-.02,.005]
  B1=np.linalg.inv(C1)@C0@B0
  def row(label,B,C):return dict(label=label,T_base_optical=C.tolist(),
     rgb={'T_optical_board':B.tolist(),'object_points_m':self.points.tolist()},
     binding={'source_publication_header_ns':1,'q_bracket_before':[1,[0]*7]})
  result=M.compare_frames(row('source',B0,C0),row('destination',B1,C1))
  np.testing.assert_allclose(result['T_camera0_camera1_RGB_only'],C1,atol=1e-12)
  self.assertLess(result['fixed_board_base_point_drift_mm']['rms'],1e-9)
  biased=C1.copy();biased[:3,3]+=[0,0,.02]
  changed=M.compare_frames(row('source',B0,C0),row('destination',B1,biased))
  self.assertAlmostEqual(changed['fixed_board_base_point_drift_mm']['rms'],20,places=8)
  self.assertEqual(result['T_camera0_camera1_RGB_only'],changed['T_camera0_camera1_RGB_only'])

 def test_binding_cannot_relabel_wrong_header_or_unbracketed_q(self):
  e=dict(source_stamp_ns=100,time_ns=150,q_source_before=[90,[0]*7],q_source_after=[110,[0]*7],span_deg=[0]*6)
  result=M.validate_binding(e,{'source_stamp_ns':np.int64(100)});self.assertEqual(result['snapshot_receipt_age_ms'],.00005)
  with self.assertRaisesRegex(ValueError,'HEADER_BINDING'):M.validate_binding(e,{'source_stamp_ns':99})
  e['q_source_after'][0]=99
  with self.assertRaisesRegex(ValueError,'BRACKET'):M.validate_binding(e,{'source_stamp_ns':100})

 def test_board_spec_is_explicit_and_no_guess_of_pattern_or_dimensions(self):
  spec=board_spec();spec.pop('legacy_pattern')
  with self.assertRaisesRegex(ValueError,'BOARD_SPEC_INCOMPLETE'):M.board_from_spec(spec)
  spec=board_spec();spec['marker_length_m']=.02
  with self.assertRaisesRegex(ValueError,'BOARD_LENGTH'):M.board_from_spec(spec)

 def test_source_clock_and_moving_feedback_quality_are_not_assumed(self):
  ns=1_000_000_000
  e=dict(source_stamp_ns=ns,time_ns=ns+20_000_000,q_source_before=[ns-300_000_000,[0]*7],
         q_source_after=[ns+300_000_000,[0]*7],span_deg=[0,0,.25,0,0,0])
  row=M.validate_binding(e,{'source_stamp_ns':np.int64(ns)})
  self.assertFalse(row['q_nearest_bracket_100ms_pass']);self.assertFalse(row['q_bracket_gap_250ms_pass'])
  self.assertFalse(row['recorded_stationary_span_0_15deg_pass'])

 def test_missing_depth_common_ids_cannot_remove_valid_RGB_comparison(self):
  B=np.eye(4);B[2,3]=.4
  first=dict(label='a',T_base_optical=np.eye(4).tolist(),
       rgb={'T_optical_board':B.tolist(),'object_points_m':self.points.tolist(),'ids':[1,2,3]},
       binding={'source_publication_header_ns':1,'q_bracket_before':[1,[0]*7]},
       depth={'depth_supported_corners_optical_m':[[.01,0,.4],[.02,0,.4],[.03,0,.4]]})
  second=dict(first,label='b');second['rgb']=dict(first['rgb'],ids=[10,11,12])
  result=M.compare_frames(first,second)
  self.assertIn('T_camera0_camera1_RGB_only',result);self.assertIn('depth_supported_motion_error',result)

 def test_pure_FK_can_verify_frozen_model_without_any_ROS_import(self):
  pure=M.extract_pure(M.GUARD,['_array','_axis_rotation','SerialUrdfFk'])
  xml='<robot name="synthetic"><link name="base_link"/>'
  for i in range(1,7):
   parent='base_link'if i==1 else 'link%d'%(i-1);child='tool0'if i==6 else 'link%d'%i
   xml+='<joint name="Joint%d" type="revolute"><parent link="%s"/><child link="%s"/><origin xyz=".1 0 0"/><axis xyz="0 0 1"/></joint>'%(i,parent,child)
  xml+='</robot>'
  fk=pure['SerialUrdfFk'](xml,['Joint%d'%i for i in range(1,7)])
  np.testing.assert_allclose(fk([0]*6)[:3,3],[.6,0,0],atol=1e-12)
  self.assertNotIn('rospy',pure)

 def test_redetection_on_generated_board_uses_RGB_without_TF_or_depth(self):
  spec=board_spec();board,_=M.board_from_spec(spec)
  if hasattr(board,'generateImage'):gray=board.generateImage((640,480),marginSize=20,borderBits=1)
  else:gray=board.draw((640,480),marginSize=20,borderBits=1)
  image=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR);detected=M.detect_rgb(image,spec)
  self.assertGreaterEqual(len(detected['ids']),35)
  self.assertGreater(len(detected['pixels']),0);self.assertEqual(len(set(detected['ids'])),len(detected['ids']))

 def test_pure_dependency_extraction_does_not_import_ROS_node(self):
  pure=M.extract_pure(M.TRACKER,['runtime_color_model_evidence','RuntimeColorModel'])
  self.assertNotIn('rospy',pure);self.assertNotIn('tf2_ros',pure)
  self.assertNotIn('CharucoTracker',pure)

 def test_collinear_board_correspondences_rejected(self):
  line=np.c_[np.arange(8)*.01,np.zeros((8,2))]
  with self.assertRaisesRegex(ValueError,'COLLINEAR'):
   M.solve_rgb(self.model,line,self.pixels[:8])

 def test_quality_selection_retains_under_count_frame_without_using_depth(self):
  row=dict(rgb={'quality_35corners_1_5px':False},binding=dict(
      q_nearest_bracket_100ms_pass=True,q_bracket_gap_250ms_pass=True,
      recorded_stationary_span_0_15deg_pass=True),depth_error='DEPTH_FAILURE')
  selection=M.frame_selection(row);self.assertFalse(selection['accepted'])
  self.assertEqual(selection['rejected_requirements'],['RGB_35corners_1_5px'])
  row['rgb']['quality_35corners_1_5px']=True
  self.assertTrue(M.frame_selection(row)['accepted'])

 def test_pair_sensitivity_keeps_all_nine_conditional_fold_combinations(self):
  B=np.eye(4);B[2,3]=.4;rgb=M.rgb_review(self.model,self.detection())
  row=dict(label='a',T_base_optical=np.eye(4).tolist(),rgb=rgb,
      binding={'source_publication_header_ns':1,'q_bracket_before':[1,[0]*7]})
  result=M.compare_frames(row,dict(row,label='b'))
  variants=result['RGB_spatial_fold_pair_sensitivity']['variants']
  self.assertEqual(len(variants),9)
  self.assertEqual({(v['first_fold'],v['second_fold'])for v in variants},
      {(a,b)for a in range(3)for b in range(3)})
  self.assertLess(max(v['fixed_board_base_point_drift_RMS_mm']for v in variants),.001)


if __name__=='__main__':unittest.main()
