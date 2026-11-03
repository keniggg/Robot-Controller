"""Static CAD union integration, frozen REC3 fragments and geometric counterexamples."""
from copy import deepcopy
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from static_cad_contract_fixtures import gg,G,XML,DATA,context,rigid,static_args,static_synthetic

@pytest.mark.parametrize('index',range(3))
@pytest.mark.parametrize('world',range(3))
@pytest.mark.parametrize('scale',[.85,1.,1.04])
def test_rec3_lateral_static_finger_false_positive_can_be_separated(index,world,scale):
    kw=static_args(index,world,scale);ctx=context()
    assert ctx is not None
    before={k:v.copy() for k,v in kw.items() if isinstance(v,np.ndarray)}
    # First bind this to the actual historical analytical-box symptom.
    legacy=gg._check_boxes(**kw)
    if scale>=1.:
        assert not legacy[0] and 'finger intrudes' in legacy[2], legacy
    result=gg._check_boxes(**kw,finger_cad_context=ctx)
    assert result[0], result
    for k,v in before.items():np.testing.assert_array_equal(kw[k],v)

@pytest.mark.parametrize('body',['left_finger','right_finger'])
@pytest.mark.parametrize('opening',[0.,.015,.03,.0498,.05])
@pytest.mark.parametrize('component',[0,1])
def test_actual_component_vertex_intersection_rejected_for_all_openings(body,opening,component):
    ctx=context();part=ctx.finger_cad_model._body_components(body)[component]
    vertices=part.vertices_at_zero+opening*np.asarray(part.velocity_per_opening)
    index=np.argmax(vertices[:,1]) if body=='left_finger' else np.argmin(vertices[:,1])
    vertex=vertices[index]
    T=rigid(1);T[:3,3]=[.1,-.2,.5]
    target=T[:3,3]+T[:3,:3]@vertex
    kw=static_synthetic(T,target,np.full(3,.0002),T[:3,:3],opening)
    # The raw, complete mesh contains this target-centred vertex. UNKNOWN is
    # not used as a penetration oracle.
    np.testing.assert_allclose(T[:3,:3].T@(target-T[:3,3]),vertex,atol=1e-15)
    result=gg._check_boxes(**kw,finger_cad_context=ctx)
    assert not result[0] and 'finger intrudes' in result[2],result

@pytest.mark.parametrize('body',['left_finger','right_finger'])
@pytest.mark.parametrize('opening',[.03,.0498,.05])
def test_active_cad_beyond_old_box_cannot_bypass_union_broadphase(body,opening):
    ctx=context();part=ctx.finger_cad_model._body_components(body)[0]
    v=part.vertices_at_zero+opening*np.asarray(part.velocity_per_opening)
    vertex=v[np.argmax(v[:,1]) if body=='left_finger' else np.argmin(v[:,1])]
    T=np.eye(4);T[:3,3]=[0.,0.,.5];target=T[:3,3]+vertex
    kw=static_synthetic(T,target,np.full(3,.0002),opening=opening)
    old=gg._check_boxes(**kw)
    assert old[0],old
    result=gg._check_boxes(**kw,finger_cad_context=ctx)
    assert not result[0] and 'finger intrudes' in result[2],result

@pytest.mark.parametrize('xml',[None,'','<robot name="unsupported"/>'])
def test_missing_or_unsupported_model_retains_original_rejection(xml):
    assert context(xml) is None
    kw=static_args()
    assert gg._check_boxes(**kw,finger_cad_context=context(xml))==gg._check_boxes(**kw)

@pytest.mark.parametrize('axes',[('x','z'),('-y','z'),('y','-z'),('z','y')])
def test_axis_mismatch_never_enables_supported_cad_context(axes):
    assert context(axes=axes) is None

@pytest.mark.parametrize('mutation',['finger_axis','finger_size','max_gap'])
def test_unvalidated_geometry_cannot_select_the_supported_binding(mutation):
    xml=XML;g=G
    if mutation=='finger_axis':
        root=ET.fromstring(XML)
        root.find("./joint[@name='left_finger']/axis").set('xyz','0 0 1')
        xml=ET.tostring(root,encoding='unicode')
    elif mutation=='finger_size':
        g=gg.GripperGeometry(.05,.002,np.array([.0434,.0287,.06]),G.palm_size_xyz_m,.003)
    else:
        g=gg.GripperGeometry(.051,.002,G.finger_size_xyz_m,G.palm_size_xyz_m,.003)
    assert context(xml,g) is None

@pytest.mark.parametrize('world',range(3))
def test_palm_overlap_retained_by_static_cad_gate(world):
    T=rigid(world);T[:3,3]+=[0.,0.,.5]
    _,center,_=next(x for x in gg._stage_boxes(T,G,.05,'y','z') if x[0]=='palm')
    kw=static_synthetic(T,center,np.full(3,.008),T[:3,:3])
    result=gg._check_boxes(**kw,finger_cad_context=context())
    assert not result[0] and 'palm intrudes' in result[2],result

def test_original_support_clearance_retained():
    T=np.eye(4);T[:3,3]=[0.,0.,.001]
    kw=static_synthetic(T,[2.,2.,2.],[.01,.01,.01]);kw['support_offset']=0.
    assert gg._check_boxes(**kw)==gg._check_boxes(**kw,finger_cad_context=context())
    assert 'support clearance' in gg._check_boxes(**kw)[2]

@pytest.mark.parametrize('body',['left_finger','right_finger'])
def test_shared_static_and_path_overlap_have_identical_padded_scene_semantics(body):
    from alicia_flexible_grasp.robot import observation_path_guard as path
    kw=static_args();T=kw['transform'];ctx=context()
    _,center,size=next(x for x in gg._stage_boxes(T,G,kw['opening_width_m'],'y','z') if x[0]==body)
    scene=NS(center=kw['obb_center'],rotation=kw['obb_rotation'],size=kw['obb_size'])
    a=gg.gripper_target_overlap(ctx,G,kw['opening_width_m'],body,T,center,size,scene)
    b=path.gripper_target_overlap(ctx,G,kw['opening_width_m'],body,T,center,size,scene)
    assert a==b

