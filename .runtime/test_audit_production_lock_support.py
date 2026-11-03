import ast
import copy
from pathlib import Path
import numpy as np
import pytest
import audit_production_lock_support as a


def fixture():
    ns=2_000_000_001
    row=dict(source_stamp_ns=ns,selected_receipt_ns=ns+100,board_required=True,current_board_observed=True,
             receipts_ns={k:ns+100 for k in a.REQUIRED},generation=3,selection_stamp_ns=1,
             anchor_source_stamp_ns=ns-1,camera_model_fingerprint='same')
    s=dict(state='ready',target_locked=True,target_lost=False,exact_rgbd=True,
           **{k:ns for k in ('source_stamp_ns','color_stamp_ns','depth_stamp_ns','tf_lookup_stamp_ns')},
           generation=3,selection_stamp_ns=1,anchor_source_stamp_ns=ns-1)
    q=dict(source_stamp_ns=ns,pose_ok=True,charuco_count=76,edge_clearance_px=25,reprojection_rms_px=.133,
           board_size=[12,9],dictionary_id='DICT_5X5_100',square_length_m=.015,camera_model_fingerprint='same')
    m={k:object() for k in a.REQUIRED};m.update(status=s,quality=q,_bag_receipts_ns=dict(row['receipts_ns']))
    return row,m


def test_exact_selected_source_and_post_model_cutoff():
    row,m=fixture()
    assert a.validate_source(row,m,1_000_000_000)==row['source_stamp_ns']
    with pytest.raises(ValueError,match='boundary'):a.validate_source(row,m,0,row['source_stamp_ns'])
    m['status']['tf_lookup_stamp_ns']-=1
    with pytest.raises(ValueError,match='source mismatch'):a.validate_source(row,m,0)


@pytest.mark.parametrize('change', ['missing','receipt','identity','lost','board'])
def test_incomplete_or_changed_selected_tuple_rejected(change):
    row,m=fixture()
    if change=='missing':del m['depth']
    if change=='receipt':row['receipts_ns']['board']+=350_000_000
    if change=='identity':m['status']['generation']+=1
    if change=='lost':m['status']['target_lost']=True
    if change=='board':m['quality']['square_length_m']=.016
    with pytest.raises(ValueError):a.validate_source(row,m,0)


def test_adjacent_ring_excludes_full_board_margin_and_object_pixels():
    p=np.array([[.09,.14,0],[.09,.141,0],[.09,.142,0],[.12,.161,0],[.12,.161,0]])
    valid=np.ones(5,dtype=bool);masked=np.array([0,0,0,1,0],dtype=bool)
    ring,board=a.adjacent_region(p,[.09,.185,0],valid,masked)
    assert board.tolist()==[True,True,False,False,False]
    assert ring.tolist()==[False,False,True,False,True]


def test_four_mm_four_degree_unchanged_and_nonfinite_rejected():
    assert a.support_pass(dict(normal_deg=4.,max_footprint_difference_mm=4.))
    assert not a.support_pass(dict(normal_deg=0.,max_footprint_difference_mm=5.4))
    assert not a.support_pass(dict(normal_deg=4.01,max_footprint_difference_mm=0.))
    assert not a.support_pass(dict(normal_deg=float('nan'),max_footprint_difference_mm=0.))


def test_stationarity_requires_real_window_not_only_last_pose():
    rows=[(i*100_000_000,np.zeros(6)) for i in range(1,9)]
    assert a.stationary_window(rows,800_000_000)['count']==8
    moved=copy.deepcopy(rows);moved[0][1][3]=np.radians(.16)
    with pytest.raises(ValueError,match='stationary'):a.stationary_window(moved,800_000_000)
    with pytest.raises(ValueError):a.stationary_window(rows[-5:],800_000_000)


def test_entry_has_no_live_ros_or_command_api():
    tree=ast.parse(Path(a.__file__).read_text())
    calls={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
    assert not calls & {'Publisher','ServiceProxy','publish','init_node','set_param','wait_for_message'}


def test_full_depth_audit_rejects_elevated_board_as_motor_support(tmp_path,monkeypatch):
    import types
    from cv_bridge import CvBridge
    from geometry_msgs.msg import TransformStamped
    import audit_motor_adjacent_table as reused
    ci=dict(width=640,height=480,fx=400.,fy=400.,cx=320.,cy=240.)
    class Rays:
        fingerprint='fixture_rays'
        intrinsics=types.SimpleNamespace(width=640,height=480)
        def normalized_points(self,uv):
            uv=np.asarray(uv);return (uv-[320.,240.])/400.
    monkeypatch.setattr(reused.D.CM,'RuntimeColorModel',lambda _e:Rays())
    image=np.zeros((480,640,3),dtype=np.uint8)
    depth=np.full((480,640),3054,dtype=np.uint16)
    yy,xx=np.indices(depth.shape)
    boardx=(xx-320.)*.3/400.+.09
    boardy=(yy-240.)*.3/400.+.11
    on_board=(boardx>=0)&(boardx<=.18)&(boardy>=0)&(boardy<=.135)
    depth[on_board]=3000
    mask=(xx>=313)&(xx<=327)&(yy>=314)&(yy<=334)
    depth[mask]=2854
    bridge=CvBridge();t=TransformStamped()
    # R maps board x,y,z onto ROS camera -y,-z,+x.
    t.transform.rotation.x=.5;t.transform.rotation.y=-.5;t.transform.rotation.z=.5;t.transform.rotation.w=-.5
    t.transform.translation.x=.3;t.transform.translation.y=.09;t.transform.translation.z=.11
    assert np.allclose(a.transform_matrix(t)[:3,:3],reused.D.O[:3,:3])
    q=dict(camera_model={'color_intrinsics':ci},camera_model_fingerprint='fixture_rays')
    m=dict(rgb=bridge.cv2_to_imgmsg(image,'bgr8'),depth=bridge.cv2_to_imgmsg(depth,'16UC1'),
           mask=bridge.cv2_to_imgmsg(mask.astype(np.uint8)*255,'mono8'),quality=q,board=t,
           status={'geometry_metrics':{'support_plane_base':[-1.,0.,0.,.3]}},object=types.SimpleNamespace(label='unknown_small'))
    camera=dict(depth_scale=.0001,runtime_profile={'color_intrinsics':ci,'geometry_intrinsics':ci})
    result=a.audit_frame(m,camera,np.eye(4),tmp_path,100)
    assert not result['production_support_consistent']
    assert result['production_support_matches_board_top']
    assert result['production_vs_adjacent_table']['max_footprint_difference_mm']==pytest.approx(5.4,abs=1e-8)
    assert result['motor_height_p10_p50_p90_mm'][1]==pytest.approx(20.,abs=1e-8)
    data=np.load(tmp_path/'100_measured.npz')
    assert not np.any(data['adjacent_table_region'] & data['board_excluded'])
    m['status']['geometry_metrics']['support_plane_base']=[-1.,0.,0.,.3054]
    accepted=a.audit_frame(m,camera,np.eye(4),tmp_path,101)
    assert accepted['production_support_consistent']
    assert not accepted['production_support_matches_board_top']


def test_offline_tf_buffer_does_not_contact_ros_master():
    tree=ast.parse(Path(a.__file__).read_text())
    buffers=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='Buffer']
    assert len(buffers)==1
    assert any(k.arg=='debug' and isinstance(k.value,ast.Constant) and k.value.value is False for k in buffers[0].keywords)
    assert 'lookup_transform_core' in Path(a.__file__).read_text()


def test_missing_edge_depth_does_not_shrink_support_query_domain():
    # Only the centre has depth. Tilt agrees centrally but differs by 5 mm at
    # the silhouette boundary. The full-mask domain must expose that mismatch.
    measured=np.tile([.3,0.,0.],(60,1))
    rays=np.array([[1.,0.,0.],[1.,0.,.4]])
    table=np.array([-1.,0.,0.,.3])
    complete,orthogonal,silhouette=a.complete_support_footprint(measured,rays,table)
    theta=np.arctan(.005/.12)
    production=np.array([-np.cos(theta),0.,np.sin(theta),.3*np.cos(theta)])
    assert abs(orthogonal@production[:3]+production[3]).max()<1e-10
    assert abs(complete@production[:3]+production[3]).max()>.004
    assert len(silhouette)==len(rays)
    with pytest.raises(ValueError):a.complete_support_footprint(measured,[[0.,1.,0.]],table)


def test_missing_or_preselection_motor_anchor_is_incomplete():
    row,m=fixture()
    for anchor in (None,0,row['source_stamp_ns']+1):
        row['anchor_source_stamp_ns']=anchor;m['status']['anchor_source_stamp_ns']=anchor
        with pytest.raises(ValueError,match='anchor'):a.validate_source(row,m,0)


@pytest.mark.parametrize('bad', ['wrong_bag_receipt','duplicate','delayed_selection'])
def test_bag_receipt_identity_is_not_replaced_by_sidecar_claim(bad):
    row,m=fixture()
    if bad=='wrong_bag_receipt':m['_bag_receipts_ns']['depth']+=1
    if bad=='duplicate':m['_duplicate_selected']=['rgb']
    if bad=='delayed_selection':row['selected_receipt_ns']+=350_000_000
    with pytest.raises(ValueError):a.validate_source(row,m,0)
