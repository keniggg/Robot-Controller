"""Frozen offline fixtures; no ROS graph, scene acquisition, or execution."""
from pathlib import Path
import gzip
import hashlib
import importlib.util
import json
import sys
from copy import deepcopy

D = Path(__file__).resolve().parent
ROOT = D.parent
for path in (ROOT, ROOT / 'src', D):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import numpy as np
from alicia_flexible_grasp.grasp import gripper_geometry as gg
from tf.transformations import quaternion_matrix, quaternion_from_matrix, euler_matrix
from test_gripper_geometry import candidate_fixture, explicit_carton_fixture

# These are immutable fixture identities, not paths to a recording drive.
FIXTURE_COMPRESSED_SHA256 = 'f4c37139662901cdbb09828c06bcaf5f291b057dd4786884b24786032160d2b1'
FIXTURE_DECOMPRESSED_SHA256 = '7795eb33ea5971ae44031a76f886c63821b2bb227df87b336ecd4b426b30fa04'
ORIGINAL_FIXTURE_SHA256 = '20dad10f53f9b0407f91ad0c44af8d951f027a51cd2f81227c3888842c49fed7'
ROBOT_DESCRIPTION_SHA256 = '7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'
_fixture_path = D / 'fixtures' / 'static_cad_contract_rec3.json.gz'
_compressed = _fixture_path.read_bytes()
if hashlib.sha256(_compressed).hexdigest() != FIXTURE_COMPRESSED_SHA256:
    raise ValueError('static CAD compressed fixture SHA256 mismatch')
_decoded = gzip.decompress(_compressed)
if hashlib.sha256(_decoded).hexdigest() != FIXTURE_DECOMPRESSED_SHA256:
    raise ValueError('static CAD decoded fixture SHA256 mismatch')
_bundle = json.loads(_decoded)
_fixture_text = _bundle['fixture_json_utf8']
XML = _bundle['robot_description_xml']
if hashlib.sha256(_fixture_text.encode('utf-8')).hexdigest() != ORIGINAL_FIXTURE_SHA256:
    raise ValueError('static CAD original fixture SHA256 mismatch')
if hashlib.sha256(XML.encode('utf-8')).hexdigest() != ROBOT_DESCRIPTION_SHA256:
    raise ValueError('static CAD robot description SHA256 mismatch')
DATA = json.loads(_fixture_text)
G=gg.GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
def context(xml=XML,g=G,axes=('y','z')):
    return gg.supported_finger_cad_context(xml,g,*axes)
def rigid(index):
    Q=euler_matrix(*[(0.,0.,0.),(.3,-.4,.7),(-.6,.2,-1.1)][index])
    Q[:3,3]=[(0.,0.,0.),(.11,-.07,.19),(-.16,.13,.32)][index]
    return Q
def static_args(index=0,world=0,scale=1.):
    c=DATA['rows'][index]['certificate'];s=c['target_scene'];Q=rigid(world)
    T=Q@np.array(c['T_base_tool0'])
    normal=Q[:3,:3]@np.array(s['normal'])
    return dict(transform=T,gripper=G,opening_width_m=c['hull_metadata']['opening_m'],
      tool_jaw_axis='y',tool_finger_length_axis='z',support_normal=normal,
      support_offset=s['offset']-float(normal@Q[:3,3]),
      obb_center=Q[:3,:3]@np.array(s['center'])+Q[:3,3],
      obb_rotation=Q[:3,:3]@np.array(s['rotation']),
      obb_size=np.array(s['size'])*scale,allow_finger_contact=False)
def static_synthetic(T,center,size,rotation=None,opening=.05,allow=False):
    return dict(transform=T,gripper=G,opening_width_m=opening,
      tool_jaw_axis='y',tool_finger_length_axis='z',support_normal=np.array([0.,0.,1.]),
      support_offset=2.,obb_center=np.array(center),obb_rotation=np.eye(3) if rotation is None else rotation,
      obb_size=np.array(size),allow_finger_contact=allow)
def transformed_public(values,world):
    v=deepcopy(values);Q=rigid(world);R=Q[:3,:3];t=Q[:3,3]
    for k in ('candidate_center_base','candidate_tool0_base','obb_center_base'):
        v[k]=R@v[k]+t
    for k in ('R_base_tool','R_base_obb'):v[k]=R@v[k]
    for k in ('pregrasp_T_base_tool','approach_T_base_tool','grasp_T_base_tool','lift_T_base_tool'):
        v[k]=Q@v[k]
    if 'target_points_base' in v:v['target_points_base']=np.asarray(v['target_points_base'])@R.T+t
    v['support_normal_base']=R@v['support_normal_base']
    v['support_offset_m']-=float(v['support_normal_base']@t)
    return v
_remote=None
def remote():
    global _remote
    if _remote is None:
        spec=importlib.util.spec_from_file_location('static_cad_offline_remote',
            str(ROOT/'scripts/remote_grasp6d_node.py'))
        _remote=importlib.util.module_from_spec(spec);spec.loader.exec_module(_remote)
    return _remote
def core_fixture(body='left_finger',world=0):
    r=remote()
    from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
    from genpy import Time
    node=r.RemoteGrasp6DNode.__new__(r.RemoteGrasp6DNode);node.gripper_geometry=G
    p=Grasp6DPlan();p.header.frame_id='base_link';p.header.stamp=Time(1)
    p.valid=True;p.target_track_id='g0-t1';p.refinement_status='NOT_EVALUATED'
    p.score=.9;p.candidate_source='graspnet';p.candidate_source_lineage=['graspnet']
    p.has_candidate_model_width=True;p.candidate_width_m=.039;p.required_open_width_m=.044
    p.model_choice='unknown_tabletop';p.refinement_translation_m=0.;p.refinement_rotation_deg=0.
    T=np.eye(4);T[:3,3]=[0.,0.,.1];Q=rigid(world)
    c=next(c for name,c,size in gg._stage_boxes(T,G,.0495,'y','z') if name==body)
    TT=Q@T;q=quaternion_from_matrix(TT)
    p.poses=[r.make_pose_stamped('base_link',TT[:3,3],q,p.header.stamp).pose for _ in range(4)]
    o=p.object_geometry;o.header=deepcopy(p.header);o.valid=True;o.target_track_id=p.target_track_id
    o.label='carton';o.source_mode='instance_mask'
    c=Q[:3,:3]@c+Q[:3,3];qo=quaternion_from_matrix(Q)
    o.pose_base.position.x,o.pose_base.position.y,o.pose_base.position.z=c
    o.pose_base.orientation.x,o.pose_base.orientation.y,o.pose_base.orientation.z,o.pose_base.orientation.w=qo
    o.size_xyz_m.x=o.size_xyz_m.y=o.size_xyz_m.z=.008
    o.support_normal_base.x,o.support_normal_base.y,o.support_normal_base.z=Q[:3,2]
    o.support_offset_m=-float(Q[:3,2]@Q[:3,3])
    p.plan_id=r.compute_plan_id(p)
    return node,p

