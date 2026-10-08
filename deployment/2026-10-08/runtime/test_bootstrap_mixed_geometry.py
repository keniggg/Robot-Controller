"""Offline mixed L1 proof tests; no node, publisher, device or service calls."""
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
import bounded_joint_response_diagnostic as d
import bootstrap_mixed_geometry as g


def fixture():
    import pyrealsense2 as rs
    root = Path(__file__).resolve().parents[1]
    fk = d.SerialUrdfFk((root/'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text(), d.NAMES[:6])
    q = np.array([-1.9358838,.5230874,-.1319223,-.0567573,-.477068,.0490874,.04975])
    X = d.quaternion_matrix([.00457828811487,-.720538213113,.00981708210786,.693330619272]); X[:3,3]=[-.0804218708267,.00814145254385,-.115009423571]
    B = np.eye(4); B[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]; B[:3,3]=[.35,.09,.0675]
    ci=dict(width=640,height=480,fx=395.768280,fy=395.602753,cx=321.0,cy=238.0,model='inverse_brown_conrady',coeffs=[-.0537299253,.0583461151,.0021220003,.0007625872,-.0197049230])
    model = d.CAMERA.RuntimeColorModel(dict(sdk_version='2.56.5',color_intrinsics=ci),sdk=rs)
    gripper=d.GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
    geometry=dict(X=X,Y=fk(q[:6])@X@B,model=model)
    return fk,gripper,q,geometry


def bound(data, radius=2.):
    fk,gr,q,geo=data
    return g.mixed_sync_geometry(fk,gr,q,geo,fk.limits[:,0],fk.limits[:,1],radius)


def test_real_urdf_mixed_response_interior_and_extremes_are_enclosed():
    data=fixture(); fk,gr,q,geo=data; proof=bound(data)
    assert proof['minimum_plane_clearance_lower_bound_m'] >= .04
    assert proof['minimum_full_board_edge_lower_bound_px'] >= 20
    assert not proof['whole_arm_collision_certified']
    rng=np.random.default_rng(719)
    raw=rng.normal(size=(150,6)); raw/=np.abs(raw).sum(axis=1)[:,None]
    # Mix boundary and interior; axis rays alone cannot verify this contract.
    deltas=np.concatenate([raw*math.radians(2),raw*.37*math.radians(2),np.eye(6)*math.radians(2),-np.eye(6)*math.radians(2)])
    normal=geo['Y'][:3,2].copy()
    if normal[2]<0: normal*=-1
    offset=-normal@geo['Y'][:3,3]
    board=np.column_stack([rng.uniform(0,.18,31),rng.uniform(0,.135,31),np.zeros(31),np.ones(31)])
    nominal=(np.linalg.inv(fk(q[:6])@geo['X'])@geo['Y']@board.T).T[:,:3]
    rectangle=np.asarray(proof['full_board_pixel_rectangle_uv'])
    for delta in deltas:
        tool=fk(q[:6]+delta)
        clear=min(np.min(d._box_corners(c,tool[:3,:3],size)@normal+offset) for _,c,size in d._stage_boxes(tool,gr,q[6],'y','z'))
        assert clear >= proof['minimum_plane_clearance_lower_bound_m']
        points=(np.linalg.inv(tool@geo['X'])@geo['Y']@board.T).T[:,:3]
        assert np.max(np.linalg.norm(points-nominal,axis=1)) <= proof['full_board_camera_vector_displacement_upper_bound_m']
        for p in points:
            uv=geo['model'].sdk.rs2_project_point_to_pixel(geo['model'].intrinsics,[-p[1],-p[2],p[0]])
            assert np.all(uv >= rectangle[:,0]) and np.all(uv <= rectangle[:,1])


@pytest.mark.parametrize('kind',['none','inverse_brown_conrady','modified_brown_conrady','brown_conrady'])
def test_projection_interval_contains_actual_sdk_interior_and_box_corners(kind):
    _,_,_,geo=fixture(); model=geo['model']; model.evidence['color_intrinsics']['model']=kind; model.intrinsics.model=getattr(model.sdk.distortion,kind)
    lo=np.array([-.18,-.15,.22]); hi=np.array([.2,.13,.44])
    rect=g.projection_rectangle(model,lo,hi)
    rng=np.random.default_rng(83)
    points=np.concatenate([rng.uniform(lo,hi,(500,3)),np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])])
    for p in points:
        uv=model.sdk.rs2_project_point_to_pixel(model.intrinsics,p.tolist())
        assert np.all(uv>=rect[:,0]) and np.all(uv<=rect[:,1])


@pytest.mark.parametrize('mutation',['nan','depth_zero','depth_negative','reversed','bad_model','wrong_sdk','zero_focal'])
def test_invalid_projection_fails_closed(mutation):
    *_,geo=fixture(); model=geo['model']; lo=np.array([-.1,-.1,.2]); hi=np.array([.1,.1,.4])
    if mutation=='nan': lo[0]=np.nan
    if mutation=='depth_zero': lo[2]=0
    if mutation=='depth_negative': lo[2]=-.1
    if mutation=='reversed': lo[0]=.3
    if mutation=='bad_model': model.evidence['color_intrinsics']['model']='ftheta'
    if mutation=='wrong_sdk': model.backend_sdk_version='2.55.1'
    if mutation=='zero_focal': model.intrinsics.fx=0
    with pytest.raises(RuntimeError): g.projection_rectangle(model,lo,hi)


@pytest.mark.parametrize('edge,ok',[(20.,True),(20.-1e-6,False)])
def test_20_pixel_boundary_not_relaxed(monkeypatch,edge,ok):
    data=fixture()
    monkeypatch.setattr(g,'projection_corner_boxes',lambda *a: np.array([[edge,600.],[30.,440.]]))
    if ok: assert bound(data)['minimum_full_board_edge_lower_bound_px']==20.
    else:
        with pytest.raises(RuntimeError,match='40 mm plane / 20 px'): bound(data)


def test_40mm_plane_gate_and_joint_limit_cannot_be_bypassed():
    data=fixture(); first=bound(data); fk,gr,q,geo=data
    normal=geo['Y'][:3,2].copy()
    if normal[2]<0: normal*=-1
    geo['Y'][:3,3]+=normal*(first['minimum_plane_clearance_lower_bound_m']-.03999)
    with pytest.raises(RuntimeError,match='40 mm plane / 20 px'): bound(data)
    data=fixture(); fk,gr,q,geo=data
    hi=fk.limits[:,1].copy(); hi[2]=q[2]+math.radians(1.9999)
    with pytest.raises(RuntimeError,match='joint limits'): g.mixed_sync_geometry(fk,gr,q,geo,fk.limits[:,0],hi,2.)


@pytest.mark.parametrize('mutation',['nonrigid','reflection','nonfinite_q','bad_radius','bad_gripper'])
def test_invalid_geometry_fails_closed(mutation):
    data=fixture(); fk,gr,q,geo=data; radius=2.
    if mutation=='nonrigid': geo['X'][0,0]+=.1
    if mutation=='reflection': geo['Y'][:3,0]*=-1
    if mutation=='nonfinite_q': q[0]=np.nan
    if mutation=='bad_radius': radius=3.
    if mutation=='bad_gripper': q[6]=.0501
    with pytest.raises(RuntimeError): bound(data,radius)


def test_axis_rays_can_miss_mixed_response_but_global_point_bound_contains_it():
    # A serial Z then X rotation has x=sin(qZ)*sin(qX)*length for a
    # tool-local Z corner. All axis rays have x=0, yet mixed signs lower x.
    names=['Joint%d'%i for i in range(1,7)]
    xml='<robot name="mixed_counterexample"><link name="base_link"/>'
    parent='base_link'
    for i,name in enumerate(names):
        child='tool0' if i==5 else 'link%d'%i
        axis='1 0 0' if i==5 else '0 0 1'
        xml+='<link name="%s"/><joint name="%s" type="revolute"><parent link="%s"/><child link="%s"/><axis xyz="%s"/><limit lower="-3" upper="3"/></joint>'%(child,name,parent,child,axis)
        parent=child
    fk=d.SerialUrdfFk(xml+'</robot>',names)
    rho=math.radians(2.); point=np.array([0.,0.,.1,1.])
    for i in range(6):
        for sign in (-1,1):
            q=np.zeros(6); q[i]=sign*rho
            assert abs((fk(q)@point)[0]) < 1e-15
    mixed=np.zeros(6); mixed[4:]=[rho/2.,-rho/2.]
    moved=fk(mixed)@point
    assert moved[0] < -1e-5
    allowance=rho*max(fk.corner_motion_radii(point[:3]))
    assert np.linalg.norm(moved[:3]-point[:3]) <= allowance


def test_latest_board_is_reproved_before_initial_sync_publish(monkeypatch):
    from collections import deque
    import threading
    obj=d.Diagnostic.__new__(d.Diagnostic)
    obj.args=SimpleNamespace(bootstrap_sync_bound_deg=2.)
    obj.guard=lambda:None; obj.stationary=lambda _:{}; obj.started_ns=obj.epoch=1
    obj.fault=None; obj.state={'status':'PENDING:POSITIVE_ENABLE_REQUESTED'}
    obj.reference=deque(); obj.accepted=deque([d.JointState(name=d.NAMES,position=[0.]*6+[.04975])])
    obj.fk,obj.gripper,obj.lo,obj.hi=object(),object(),object(),object()
    obj.lock=threading.RLock(); obj.path=lambda *a,**k:{}; obj.driver_connection=lambda:None
    obj.snapshot=lambda label:dict(Y=np.eye(4),X=np.eye(4),label=label)
    published=[]; obj.publisher=SimpleNamespace(publish=published.append)
    calls=[]
    def mixed(*args):
        calls.append(args[3]['label'])
        if len(calls)==2: raise RuntimeError('fresh continuous geometry rejected')
        return {}
    monkeypatch.setattr(d,'mixed_sync_geometry',mixed)
    with pytest.raises(RuntimeError,match='fresh continuous geometry rejected'): obj.bootstrap_once()
    assert calls==['bootstrap_presync','bootstrap_presync_fresh']
    assert published==[]


def test_component_jacobian_matches_independent_fk_finite_difference():
    fk,_,q,geo=fixture(); rho=math.radians(2.)
    result=g.board_camera_taylor_box(fk,q[:6],geo['X'],geo['Y'],rho)
    jac=np.asarray(result['corner_jacobian_xyz_per_joint'])
    board=np.asarray([[cx,cy,0.,1.] for cx in (0.,.18) for cy in (0.,.135)])
    world=geo['Y']@board.T
    def coordinates(v): return (np.linalg.inv(fk(v)@geo['X'])@world).T[:,:3]
    for i in range(6):
        delta=np.zeros(6);delta[i]=1e-6
        observed=(coordinates(q[:6]+delta)-coordinates(q[:6]-delta))/(2e-6)
        assert np.allclose(jac[:,:,i],observed,rtol=0,atol=2e-9)


def test_global_second_derivative_bound_covers_mixed_and_repeated_terms():
    fk,_,q,geo=fixture(); rho=math.radians(2.)
    board=np.asarray([[cx,cy,0.,1.] for cx in (0.,.18) for cy in (0.,.135)])
    world=geo['Y']@board.T
    reference=g.board_camera_taylor_box(fk,q[:6],geo['X'],geo['Y'],rho)
    S=np.asarray(reference['corner_hessian_entry_vector_norm_bound_m'])
    def coordinates(v): return (np.linalg.inv(fk(v)@geo['X'])@world).T[:,:3]
    rng=np.random.default_rng(639)
    for center in (q[:6],q[:6]+rng.uniform(-.02,.02,6),q[:6]+rng.uniform(-.02,.02,6)):
        h=2e-4; base=coordinates(center)
        for i in range(6):
            ei=np.zeros(6);ei[i]=h
            repeated=(coordinates(center+ei)-2*base+coordinates(center-ei))/(h*h)
            assert np.all(np.linalg.norm(repeated,axis=1) <= S+1e-7)
            for j in range(i):
                ej=np.zeros(6);ej[j]=h
                mixed=(coordinates(center+ei+ej)-coordinates(center+ei-ej)-coordinates(center-ei+ej)+coordinates(center-ei-ej))/(4*h*h)
                assert np.all(np.linalg.norm(mixed,axis=1) <= S+1e-7)


@pytest.mark.parametrize('radius',[.05,.5,2.])
def test_taylor_remainder_holds_for_multiaxis_responses(radius):
    fk,_,q,geo=fixture();rho=math.radians(radius)
    proof=g.board_camera_taylor_box(fk,q[:6],geo['X'],geo['Y'],rho)
    board=np.asarray([[cx,cy,0.,1.] for cx in (0.,.18) for cy in (0.,.135)])
    world=geo['Y']@board.T
    nominal=np.asarray(proof['corner_nominal_xyz_m']);jac=np.asarray(proof['corner_jacobian_xyz_per_joint'])
    remainder=np.asarray(proof['corner_second_order_coordinate_allowance_m'])
    rng=np.random.default_rng(597)
    raw=rng.normal(size=(150,6));raw/=np.abs(raw).sum(axis=1)[:,None]
    for delta in raw*rho:
        actual=(np.linalg.inv(fk(q[:6]+delta)@geo['X'])@world).T[:,:3]
        error=actual-nominal-np.einsum('pki,i->pk',jac,delta)
        # The derivation bounds the vector norm, which is stronger than each
        # coordinate's final enclosure. This is not just endpoint box testing.
        assert np.all(np.linalg.norm(error,axis=1) <= remainder+2e-12)
        assert np.all(actual>=np.asarray(proof['lower_xyz_m']))
        assert np.all(actual<=np.asarray(proof['upper_xyz_m']))


@pytest.mark.parametrize('radius',[.01,.25,.75,2.])
def test_small_cell_api_uses_requested_radius_without_changing_bootstrap_policy(radius):
    fk,gr,q,geo=fixture()
    result=g.certify_local_cell_geometry(fk,gr,q,geo,fk.limits[:,0],fk.limits[:,1],radius)
    assert result['response_total_l1_bound_deg']==radius
    assert result['kind']=='continuous_six_axis_L1_local_cell_geometry'
    assert result['minimum_full_board_edge_lower_bound_px']>=20.
    assert result['minimum_plane_clearance_lower_bound_m']>=.04
    if radius!=2.:
        with pytest.raises(RuntimeError,match='explicit initial sync'):
            g.mixed_sync_geometry(fk,gr,q,geo,fk.limits[:,0],fk.limits[:,1],radius)


@pytest.mark.parametrize('radius',[0.,-.01,2.00000001,4.8,math.nan,math.inf])
def test_small_cell_radius_fails_closed(radius):
    fk,gr,q,geo=fixture()
    with pytest.raises(RuntimeError,match='at most 2 degrees'):
        g.certify_local_cell_geometry(fk,gr,q,geo,fk.limits[:,0],fk.limits[:,1],radius)


def test_cornerwise_positive_depth_hull_contains_full_tilted_board_and_sdk_distortion():
    *_,geo=fixture(); model=geo['model']
    origin=np.array([.22,.10,.075]);u=np.array([.6,-.8,0.]);v=np.array([0.,0.,-1.])
    corners=np.array([origin+x*u+y*v for x in (0.,.18) for y in (0.,.135)])
    eps=np.array([.001,.002,.0015]);lower,upper=corners-eps,corners+eps
    rect=g.projection_corner_boxes(model,lower,upper)
    a,b=lower.min(axis=0),upper.max(axis=0)
    old=g.projection_rectangle(model,[-b[1],-b[2],a[0]],[-a[1],-a[2],b[0]])
    assert rect[0,1] < old[0,1]-15. # Real loss from mixing depths is eliminated.
    rng=np.random.default_rng(391)
    for _ in range(600):
        moved=rng.uniform(lower,upper)
        weights=rng.dirichlet(np.ones(4))
        p=weights@moved
        uv=model.sdk.rs2_project_point_to_pixel(model.intrinsics,[-p[1],-p[2],p[0]])
        assert np.all(uv>=rect[:,0]) and np.all(uv<=rect[:,1])


@pytest.mark.parametrize('mutation',['shape','nan','zero_depth','negative_depth','reversed'])
def test_corner_projection_rejects_invalid_enclosures(mutation):
    *_,geo=fixture();lo=np.array([[.2,-.1,-.1],[.3,.1,-.1],[.2,-.1,.1],[.3,.1,.1]]);hi=lo+.001
    if mutation=='shape':lo=lo[:3]
    if mutation=='nan':hi[2,1]=np.nan
    if mutation=='zero_depth':lo[1,0]=0.
    if mutation=='negative_depth':lo[3,0]=-.01
    if mutation=='reversed':lo[1,1]=hi[1,1]+.01
    with pytest.raises(RuntimeError,match='corner'):g.projection_corner_boxes(geo['model'],lo,hi)


def test_taylor_preserves_corner_identity_and_legacy_union():
    fk,_,q,geo=fixture()
    result=g.board_camera_taylor_box(fk,q[:6],geo['X'],geo['Y'],math.radians(2.))
    lo=np.array(result['corner_lower_xyz_m']);hi=np.array(result['corner_upper_xyz_m'])
    assert lo.shape==hi.shape==(4,3)
    assert np.allclose(result['lower_xyz_m'],lo.min(axis=0))
    assert np.allclose(result['upper_xyz_m'],hi.max(axis=0))


def test_batch_corner_normalization_matches_separate_cells_and_covers_interiors():
    *_,geo=fixture();model=geo['model'];rng=np.random.default_rng(99)
    origin=np.array([.22,.1,.075]);u=np.array([.6,-.8,0]);v=np.array([0,0,-1])
    board=np.array([origin+x*u+y*v for x in (0.,.18) for y in (0.,.135)])
    lower=np.stack([board+rng.uniform(-.008,.008,3)-.001 for _ in range(37)])
    upper=lower+.002
    rectangle=g.projection_corner_box_batch(model,lower,upper)
    # Independently construct normalized intervals using the original scalar
    # interval arithmetic, not by invoking the batched path again.
    normalized=[]
    for lo,hi in zip(lower,upper):
        for a,b in zip(lo,hi):normalized.append(g._normalized_box([-b[1],-b[2],a[0]],[-a[1],-a[2],b[0]]))
    normalized=np.array(normalized)
    expected=g._project_normalized_rectangle(model,(normalized[:,0,0].min(),normalized[:,0,1].max()),(normalized[:,1,0].min(),normalized[:,1,1].max()))
    assert np.array_equal(rectangle,expected)
    for lo,hi in zip(lower,upper):
        for _ in range(15):
            p=rng.dirichlet(np.ones(4))@rng.uniform(lo,hi)
            uv=model.sdk.rs2_project_point_to_pixel(model.intrinsics,[-p[1],-p[2],p[0]])
            assert np.all(uv>=rectangle[:,0]) and np.all(uv<=rectangle[:,1])


@pytest.mark.parametrize('mutation',('empty','shape','nan','depth','reversed'))
def test_batch_rejects_invalid_corner_domains(mutation):
    *_,geo=fixture();lo=np.ones((3,4,3))*.2;hi=lo+.01
    if mutation=='empty':lo=lo[:0];hi=hi[:0]
    if mutation=='shape':lo=lo[:,:3]
    if mutation=='nan':hi[1,2,0]=np.nan
    if mutation=='depth':lo[0,1,0]=0
    if mutation=='reversed':lo[0,1,2]=.22
    with pytest.raises(RuntimeError,match='corner'):g.projection_corner_box_batch(geo['model'],lo,hi)
