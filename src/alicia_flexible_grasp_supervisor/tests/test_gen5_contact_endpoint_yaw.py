"""Recorded nominal endpoint rejection and non-authorizing direction coverage."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from tf.transformations import quaternion_matrix,rotation_matrix
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.grasp.gripper_geometry import _check_boxes
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import TabletopGeometryConfig,_sample_angles,generate_tabletop_proposals
from test_tabletop_geometry_candidates import carton_result

ROWS=json.loads((Path(__file__).parent/'fixtures/gen5_request41_contact_endpoints.json').read_text())['candidates']

@pytest.mark.parametrize('row',ROWS,ids=lambda r:r['plan_id'])
def test_recorded_requested_goal_exposes_contact_vs_open_collision_policy(row):
 g=contact_precheck_gripper({});obj=row['object_geometry'];e=json.loads(row['contact_geometry_evidence'])
 vals=row['poses_xyz_xyzw'][2];T=quaternion_matrix(vals[3:]);T[:3,3]=vals[:3]
 G=quaternion_matrix(obj['pose_xyz_xyzw'][3:]);G[:3,3]=obj['pose_xyz_xyzw'][:3]
 n=np.array(obj['support_normal_base']);size=np.array(obj['size_xyz_m']);opening=row['precheck']['opening_m']
 def check(t,allow):
  return _check_boxes(t,g,opening,'y','z',n,obj['support_offset_m'],G[:3,3],G[:3,:3],size,allow,.0005)
 assert check(T,True)[0]  # Current analytical grasp-contact rule admits it.
 assert not check(T,False)[0]  # The unchanged open approach rule rejects it.
 assert check(T,False)[1]>.010  # This is target collision, not table collision.
 config=TabletopGeometryConfig(angle_step_deg=5.,angle_dedup_deg=1.)
 original=_sample_angles(G[:3,:3],n,config)
 refined=_sample_angles(G[:3,:3],n,config,True)
 added=[axis for _,axis in refined if not any(abs(axis@old)>1-1e-10 for _,old in original)]
 assert 0<len(added)<=4
 free=[]
 for axis in added:
  if axis@T[:3,1]<0:axis=-axis
  theta=float(np.arctan2(n@np.cross(T[:3,1],axis),T[:3,1]@axis))
  if abs(np.rad2deg(theta))>5.:continue
  rot=rotation_matrix(theta,n)[:3,:3];trial=T.copy();center=np.array(e['contact_center_base'])
  trial[:3,:3]=rot@T[:3,:3];trial[:3,3]=center+rot@(T[:3,3]-center)
  if check(trial,False)[0]:free.append(float(np.rad2deg(theta)))
 assert free  # Endpoint-only opportunity; no old bilateral evidence reused.
 assert all(abs(v)<5. for v in free)


def test_refinement_without_bound_measured_surface_does_not_make_candidates():
 result=carton_result(refine_obb_neighbor_angles=True)
 assert not result.ok and result.failure_code=='TABLETOP_GEOMETRY_INPUT_INVALID'


@pytest.mark.parametrize('strategy,phase,state,near,mode,expected', [
 ('observe_return','observing_contact','qualified',True,'unknown',True),
 ('direct','observing_contact','qualified',True,'unknown',False),
 ('observe_return','initial_contact','qualified',True,'unknown',False),
 ('observe_return','observing_contact','pending',True,'unknown',False),
 ('observe_return','observing_contact',None,True,'unknown',False),
 ('observe_return','observing_contact','qualified',False,'unknown',False),
 ('observe_return','observing_contact','qualified',True,'carton',False),
 ('observe_return','returned_contact','qualified',True,'unknown',False),
])
def test_refinement_scope_requires_qualified_supplement_not_multiframe(
        monkeypatch,strategy,phase,state,near,mode,expected):
 import importlib.util,sys
 import test_remote_grasp6d_streaming as f
 monkeypatch.setitem(sys.modules,'_alicia_original_remote_grasp6d',f.remote_node)
 script=Path(__file__).resolve().parents[2]/'alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'
 spec=importlib.util.spec_from_file_location('midpoint_mode_scope_test',script)
 module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 node=module.ModeAwareRemoteGrasp6DNode.__new__(module.ModeAwareRemoteGrasp6DNode)
 node._mode_selection={'mode':mode,'strategy':strategy}
 node._observe_return_phase=phase;node.near_field_planning_active=near
 node._measurement_remote_request={} if state is None else {'state':state}
 # Multiple same-station snapshots must not itself enable the extension.
 node._near_field_fused_surface=type('Surface',(),{'view_stamps_ns':(1,2,3)})()
 calls=[]
 def capture(self,*args,**kwargs):
  calls.append(kwargs);return (),{}
 monkeypatch.setattr(f.remote_node.RemoteGrasp6DNode,'_generate_tabletop_candidates',capture)
 node._generate_tabletop_candidates(object(),contact_execution_phase=True)
 assert len(calls)==1
 assert calls[0] == ({'refine_obb_neighbor_angles': True} if expected else {})
