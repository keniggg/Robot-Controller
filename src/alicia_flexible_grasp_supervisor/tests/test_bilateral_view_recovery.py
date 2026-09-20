"""Complementary view geometry; admission still requires strict MoveIt."""
import numpy as np
import pytest

from test_observation_tilt_repair import recorded_views, remote
from test_grasp_task_sequence import grasp_task_node as task


@pytest.mark.parametrize('world_yaw_deg', [0., 73., -137.])
def test_bilateral_recovery_changes_jaw_hemisphere_without_changing_range_or_mount(world_yaw_deg):
    data, node, geometry, ordered = recorded_views()
    target = geometry.center_base
    tool = remote.pose_matrix(ordered[0]['sequence'].pregrasp)
    mount = node._tool_from_camera_matrix()
    camera = tool.dot(mount)
    rotation = remote.quaternion_matrix(remote.quaternion_from_euler(0., 0., np.deg2rad(world_yaw_deg)))
    tool, camera = rotation.dot(tool), rotation.dot(camera)
    target = rotation[:3, :3].dot(target)
    support = rotation[:3, :3].dot(geometry.support_normal_base)
    jaw = rotation[:3, :3].dot(geometry.axes_base[:, 1])
    def pose(matrix):
        return remote.make_pose_stamped('base_link', matrix[:3, 3], remote.quaternion_from_matrix(matrix),
                                        stamp=remote.rospy.Time(1))
    kwargs = dict(current_pose=pose(tool), target_center_base=target, support_normal_base=support,
        lateral_offset_m=.04, radial_retreat_m=0., minimum_contact_clearance_m=.08,
        current_camera_pose=pose(camera), envelope_radius_m=.025, opening_width_m=data['opening_m'],
        gripper_geometry=node.gripper_geometry, camera_distance_band_m=(.185, .215))
    current_projection = np.dot(camera[:3, 3]-target, jaw)
    old = task.make_clear_view_reacquisition_poses(**kwargs)
    assert all(np.dot(remote.pose_matrix(p).dot(mount)[:3, 3]-target, jaw)*current_projection > 0. for p in old)
    directed = task.make_clear_view_reacquisition_poses(bilateral_jaw_axis_base=jaw, **kwargs)
    assert 1 <= len(directed) <= 2
    for p in directed:
        final_camera = remote.pose_matrix(p).dot(mount)
        ray = final_camera[:3, 3]-target
        assert np.dot(ray, jaw)*current_projection < 0.
        assert np.linalg.norm(ray) == pytest.approx(.215)
        np.testing.assert_allclose(final_camera[:3, :3].T.dot(-ray), [.215, 0., 0.], atol=1e-10)
        assert np.dot(ray/np.linalg.norm(ray), support) == pytest.approx(
            np.dot((camera[:3, 3]-target)/np.linalg.norm(camera[:3, 3]-target), support))


def test_bilateral_recovery_rejects_an_axis_normal_to_table():
    _, node, geometry, ordered = recorded_views()
    tool = remote.pose_matrix(ordered[0]['sequence'].pregrasp)
    camera = tool.dot(node._tool_from_camera_matrix())
    camera_pose = remote.make_pose_stamped('base_link', camera[:3, 3], remote.quaternion_from_matrix(camera), stamp=remote.rospy.Time(1))
    with pytest.raises(ValueError, match='support plane'):
        task.make_clear_view_reacquisition_poses(ordered[0]['sequence'].pregrasp,
            geometry.center_base, geometry.support_normal_base, lateral_offset_m=.04,
            radial_retreat_m=0., current_camera_pose=camera_pose,
            camera_distance_band_m=(.185, .215), bilateral_jaw_axis_base=geometry.support_normal_base)


@pytest.mark.parametrize('world_yaw_deg', [0.,73.,-137.])
def test_optical_roll_search_preserves_camera_ray_and_opposite_surface(world_yaw_deg):
    data,node,geometry,ordered=recorded_views()
    mount=node._tool_from_camera_matrix()
    rotation=remote.quaternion_matrix(remote.quaternion_from_euler(0.,0.,np.deg2rad(world_yaw_deg)))
    tool=rotation@remote.pose_matrix(ordered[0]['sequence'].pregrasp); camera=tool@mount
    target=rotation[:3,:3]@geometry.center_base; normal=rotation[:3,:3]@geometry.support_normal_base
    jaw=rotation[:3,:3]@geometry.axes_base[:,1]
    def pose(m): return remote.make_pose_stamped('base_link',m[:3,3],remote.quaternion_from_matrix(m),stamp=remote.rospy.Time(1))
    kwargs=dict(current_pose=pose(tool),target_center_base=target,support_normal_base=normal,
        current_camera_pose=pose(camera),lateral_offset_m=.04,radial_retreat_m=0.,
        minimum_contact_clearance_m=.08,envelope_radius_m=.025,opening_width_m=data['opening_m'],
        gripper_geometry=node.gripper_geometry,camera_distance_band_m=(.185,.215),bilateral_jaw_axis_base=jaw)
    baseline=task.make_clear_view_reacquisition_poses(**kwargs)
    expanded=task.make_clear_view_reacquisition_poses(camera_roll_offsets_deg=(0.,15.,-15.,30.,-30.,60.,-60.,90.,-90.,180.),**kwargs)
    assert len(expanded)>len(baseline)
    centres=[remote.pose_matrix(p)@mount for p in baseline]
    for p in expanded:
        view=remote.pose_matrix(p)@mount
        assert min(np.linalg.norm(view[:3,3]-c[:3,3]) for c in centres)<1e-10
        ray=target-view[:3,3]
        np.testing.assert_allclose(view[:3,:3].T@ray,[.215,0.,0.],atol=1e-10)
        assert np.dot(-ray,jaw)*np.dot(camera[:3,3]-target,jaw)<0.
    for a,b in zip(baseline,expanded):
        np.testing.assert_allclose(remote.pose_matrix(a),remote.pose_matrix(b),atol=1e-12)


@pytest.mark.parametrize('rolls', [(),(True,),('nan',),(1.,),(0.,0.),tuple(range(14)),(0.,181.)])
def test_invalid_roll_search_cannot_generate_a_candidate(rolls):
    _,node,geometry,ordered=recorded_views()
    tool=ordered[0]['sequence'].pregrasp
    matrix=remote.pose_matrix(tool)@node._tool_from_camera_matrix()
    camera=remote.make_pose_stamped('base_link',matrix[:3,3],remote.quaternion_from_matrix(matrix),stamp=remote.rospy.Time(1))
    with pytest.raises(ValueError,match='roll search'):
        task.make_clear_view_reacquisition_poses(tool,geometry.center_base,geometry.support_normal_base,
            current_camera_pose=camera,camera_roll_offsets_deg=rolls)


@pytest.mark.parametrize('yaw', [0., 73., -137.])
def test_azimuth_search_covers_opposite_hemisphere_in_object_frame(yaw):
    data, node, geometry, ordered = recorded_views()
    rotation = remote.quaternion_matrix(remote.quaternion_from_euler(0., 0., np.deg2rad(yaw)))
    mount = node._tool_from_camera_matrix()
    tool = rotation @ remote.pose_matrix(ordered[0]['sequence'].pregrasp)
    camera = tool @ mount
    target = rotation[:3, :3] @ geometry.center_base
    normal = rotation[:3, :3] @ geometry.support_normal_base
    jaw = rotation[:3, :3] @ geometry.axes_base[:, 1]
    def pose(m):
        return remote.make_pose_stamped('base_link', m[:3, 3], remote.quaternion_from_matrix(m),
                                       stamp=remote.rospy.Time(1))
    angles = (-75., 75., -45., 45., -15., 15.)
    candidates = task.make_clear_view_reacquisition_poses(pose(tool), target, normal,
        current_camera_pose=pose(camera), lateral_offset_m=.04, radial_retreat_m=0.,
        minimum_contact_clearance_m=.08, envelope_radius_m=.025, opening_width_m=data['opening_m'],
        gripper_geometry=node.gripper_geometry, camera_distance_band_m=(.185, .215),
        bilateral_jaw_axis_base=jaw, bilateral_azimuth_offsets_deg=angles)
    assert len(candidates) == len(angles)
    across = np.cross(normal, jaw)
    opposite = -np.sign(np.dot(camera[:3, 3] - target, jaw))
    for candidate, angle in zip(candidates, angles):
        view = remote.pose_matrix(candidate) @ mount
        ray = view[:3, 3] - target
        assert np.linalg.norm(ray) == pytest.approx(.215)
        np.testing.assert_allclose(view[:3, :3].T @ -ray, [.215, 0., 0.], atol=1e-10)
        assert np.dot(ray, jaw) * opposite > 0.
        assert np.rad2deg(np.arctan2(np.dot(ray, across), opposite*np.dot(ray, jaw))) == pytest.approx(angle)


@pytest.mark.parametrize('angles', [(), (True,), (float('nan'),), (90.,), (-90.,), (0., 0.), tuple(range(14))])
def test_invalid_azimuth_search_cannot_generate_a_candidate(angles):
    _, node, geometry, ordered = recorded_views()
    tool = ordered[0]['sequence'].pregrasp
    matrix = remote.pose_matrix(tool) @ node._tool_from_camera_matrix()
    camera = remote.make_pose_stamped('base_link', matrix[:3, 3], remote.quaternion_from_matrix(matrix), stamp=remote.rospy.Time(1))
    with pytest.raises(ValueError, match='azimuth'):
        task.make_clear_view_reacquisition_poses(tool, geometry.center_base, geometry.support_normal_base,
            current_camera_pose=camera, camera_distance_band_m=(.185, .215), radial_retreat_m=0.,
            bilateral_jaw_axis_base=geometry.axes_base[:, 1], bilateral_azimuth_offsets_deg=angles)
