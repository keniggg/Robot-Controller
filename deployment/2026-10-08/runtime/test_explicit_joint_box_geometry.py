"""Offline continuous box proof regressions; no node/device/service calls."""
import itertools,math
import numpy as np
import pytest
from test_bootstrap_mixed_geometry import fixture
import bootstrap_mixed_geometry as g
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes,_box_corners


def prove(data,half):
 fk,gr,q,scene=data
 return g.certify_joint_box_geometry(fk,gr,q,scene,fk.limits[:,0],fk.limits[:,1],half)


@pytest.mark.parametrize('half',[[0]*5,[0]*7,[0,0,0,0,0,float('nan')],[0,0,0,0,0,-1e-10],[math.radians(.34)]*6])
def test_invalid_box_widths_fail_closed(half):
 with pytest.raises(RuntimeError,match='half-widths'):prove(fixture(),half)


def test_axiswise_first_order_is_exact_linear_box_support_not_L1_ball():
 fk,gr,q,scene=fixture();h=np.radians([.3,.2,.03,.1,.01,0])
 t=g.board_camera_box_taylor_bound(fk,q[:6],scene['X'],scene['Y'],h)
 J=np.asarray(t['corner_jacobian_xyz_per_joint']);first=np.asarray(t['corner_first_order_coordinate_allowance_m'])
 corners=np.array(list(itertools.product((-1.,1.),repeat=6)))*h
 linear=np.einsum('pki,ni->npk',J,corners)
 np.testing.assert_allclose(first,linear.max(axis=0),atol=1e-15)
 np.testing.assert_allclose(first,-linear.min(axis=0),atol=1e-15)
 old=g.board_camera_taylor_box(fk,q[:6],scene['X'],scene['Y'],sum(h))
 assert np.all(first<=np.asarray(old['corner_first_order_coordinate_allowance_m'])+1e-15)
 assert np.max(np.asarray(old['corner_first_order_coordinate_allowance_m'])-first)>.001


def test_same_L1_radius_different_axis_widths_remain_distinct():
 fk,_,q,scene=fixture();h=np.radians([.3,0,0,0,0,0]);other=h[::-1]
 a=g.board_camera_box_taylor_bound(fk,q[:6],scene['X'],scene['Y'],h)
 b=g.board_camera_box_taylor_bound(fk,q[:6],scene['X'],scene['Y'],other)
 assert a['joint_l1_radius_rad']==b['joint_l1_radius_rad']
 assert not np.allclose(a['corner_first_order_coordinate_allowance_m'],b['corner_first_order_coordinate_allowance_m'])


def test_zero_width_point_and_limits_do_not_acquire_ball_radius():
 data=fixture();fk,gr,q,scene=data;h=np.zeros(6)
 proof=g.certify_joint_box_geometry(fk,gr,q,scene,q[:6],q[:6],h)
 assert proof['joint_box_lower_rad']==proof['joint_box_upper_rad']==q[:6].tolist()
 np.testing.assert_array_equal(proof['camera_coordinate_taylor_bound']['corner_first_order_coordinate_allowance_m'],np.zeros((4,3)))
 h[2]=1e-9
 with pytest.raises(RuntimeError,match='joint limits'):g.certify_joint_box_geometry(fk,gr,q,scene,q[:6],q[:6],h)


def test_real_urdf_box_boundary_and_interior_cad_board_projection_are_enclosed():
 data=fixture();fk,gr,q,scene=data;h=np.radians([.2,.3,.3,.02,.2,.005]);p=prove(data,h)
 rng=np.random.RandomState(422);deltas=np.vstack((np.array(list(itertools.product((-1.,1.),repeat=6)))*h,rng.uniform(-h,h,(180,6))))
 normal=scene['Y'][:3,2].copy()
 if normal[2]<0:normal=-normal
 offset=-normal@scene['Y'][:3,3]
 board=np.column_stack((rng.uniform(0,.18,13),rng.uniform(0,.135,13),np.zeros(13),np.ones(13)))
 board=np.vstack((board,np.array([[x,y,0,1] for x in (0,.18) for y in (0,.135)])))
 pixel=np.array(p['full_board_pixel_rectangle_uv']);t=p['camera_coordinate_taylor_bound'];lo=np.array(t['corner_lower_xyz_m']);hi=np.array(t['corner_upper_xyz_m'])
 for delta in deltas:
  tool=fk(q[:6]+delta)
  clearance=min(float(np.min(_box_corners(c,tool[:3,:3],size)@normal+offset)) for _,c,size in _stage_boxes(tool,gr,q[6],'y','z'))
  assert clearance>=p['minimum_plane_clearance_lower_bound_m']
  camera=(np.linalg.inv(tool@scene['X'])@scene['Y']@board.T).T[:,:3]
  assert np.all(camera[-4:]>=lo) and np.all(camera[-4:]<=hi)
  for point in camera:
   uv=scene['model'].sdk.rs2_project_point_to_pixel(scene['model'].intrinsics,[-point[1],-point[2],point[0]])
   assert np.all(uv>=pixel[:,0]) and np.all(uv<=pixel[:,1])


@pytest.mark.parametrize('edge,ok',[(20.,True),(19.999999,False)])
def test_original_twenty_pixel_boundary_is_unchanged(monkeypatch,edge,ok):
 monkeypatch.setattr(g,'projection_corner_boxes',lambda *a:np.array([[edge,600.],[30.,440.]]))
 if ok:assert prove(fixture(),np.radians([.1]*6))['minimum_full_board_edge_lower_bound_px']==20
 else:
  with pytest.raises(RuntimeError,match='40 mm plane / 20 px'):prove(fixture(),np.radians([.1]*6))


def test_original_forty_mm_plane_boundary_remains_a_hard_gate():
 data=fixture();h=np.radians([.1]*6);p=prove(data,h);fk,gr,q,scene=data
 n=scene['Y'][:3,2].copy()
 if n[2]<0:n=-n
 scene['Y'][:3,3]+=n*(p['minimum_plane_clearance_lower_bound_m']-.039999)
 with pytest.raises(RuntimeError,match='40 mm plane / 20 px'):prove(data,h)
