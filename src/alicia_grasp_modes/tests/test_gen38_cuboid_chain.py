"""One-observation handoff with an independently ray-cast visible box side.

Source pixels/depth come from slab intersections, not the production cuboid
projector. The unchanged GEN35 chain owns task, registration, candidate,
admission and closing logic. Its transport/controller/IK/physics providers
remain explicit simulators: this is no evidence of a physical grasp.
"""
from types import SimpleNamespace as NS

import numpy as np
import pytest

import test_gen35_minimal_chain_replay as chain
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


def cuboid_scene(*, width=.030, length=.065, tilt=15.):
    height = .025
    h, w = 240, 320
    K = NS(width=w, height=h, fx=600., fy=600., cx=(w-1)/2., cy=(h-1)/2.)
    angle = np.radians(tilt)
    Ry = np.array([[np.cos(angle), 0., np.sin(angle)], [0., 1., 0.],
                   [-np.sin(angle), 0., np.cos(angle)]])
    T = np.eye(4)
    T[:3, :3] = Ry @ np.diag([1., -1., -1.])
    T[:3, 3] = [.35*np.sin(angle), 0., .35*np.cos(angle)]
    yaw = np.radians(21.)
    axes = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                     [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]])
    yy, xx = np.mgrid[:h, :w]
    rays = np.stack(((xx-K.cx)/K.fx, (yy-K.cy)/K.fy, np.ones((h, w))), axis=-1) @ T[:3, :3].T
    center = np.array([0., 0., height/2.])
    origin = T[:3, 3]
    local_origin = (origin-center) @ axes
    local_rays = rays @ axes
    half = np.array([length, width, height])/2.
    with np.errstate(divide='ignore', invalid='ignore'):
        a, b = (-half-local_origin)/local_rays, (half-local_origin)/local_rays
    near, far = np.max(np.minimum(a, b), axis=-1), np.min(np.maximum(a, b), axis=-1)
    support = -origin[2]/rays[:, :, 2]
    mask = (far >= near) & (near > 0.) & (near <= support)
    distance = np.where(mask, near, support)
    depth = np.rint(distance/.0001).astype(np.uint16)
    measured = origin+rays*(depth*.0001)[:, :, None]
    local = (measured-center) @ axes
    top = mask & (np.abs(local[:, :, 2]-height/2.) < .0002)
    side = mask & ~top
    # Leave the complete RGB silhouette intact. A real missing top-depth band
    # prevents depth hull corners alone from qualifying the footprint.
    unknown = top & (local[:, :, 1] > width/2.-.003)
    depth[unknown] = 0
    retained = measured[mask & ~unknown]
    geometry = GeometryEstimate(True, '', '', center, axes,
        np.array([length, width, height]), np.array([0., 0., 1.]), 0., 1.,
        retained, 'instance_mask')
    return NS(depth=depth, mask=mask.astype(np.uint8)*255, K=K, T=T,
        geometry=geometry, identity=TargetTrackIdentity(1, 'g4-t1'),
        stamp=123_456_789_012, top=measured, local=local,
        visible_top=top, visible_side=side, unknown=unknown)


@pytest.fixture(autouse=True)
def no_unsimulated_external_calls(monkeypatch):
    import socket
    import rospy

    def forbidden(*args, **kwargs):
        raise chain.UnsimulatedExternalCall('GEN38 unsimulated boundary '+repr(args))

    for name in ('connect', 'connect_ex', 'sendto', 'bind', 'listen'):
        monkeypatch.setattr(socket.socket, name, forbidden)
    monkeypatch.setattr(socket, 'create_connection', forbidden)
    for name in ('init_node', 'Publisher', 'Subscriber', 'Service', 'ServiceProxy',
                 'wait_for_service', 'set_param', 'delete_param'):
        monkeypatch.setattr(rospy, name, forbidden)


def test_independent_slab_source_contains_real_visible_side_and_missing_top_depth():
    s = cuboid_scene()
    assert s.visible_side.sum() > 100
    assert s.visible_top.sum() > 1000
    assert s.unknown.sum() > 100
    assert np.all(s.depth[s.unknown] == 0)
    p = s.geometry.object_points_base
    local = (p-s.geometry.center_base) @ s.geometry.axes_base
    assert np.any(local[:, 2] < .008)
    assert np.all(np.abs(local) <= s.geometry.size_xyz_m/2.+.0002)


@pytest.mark.parametrize('order', [
    ('status', 'ack', 'object', 'mask'),
    ('mask', 'object', 'status', 'ack'),
    ('ack', 'mask', 'status', 'object'),
])
def test_visible_side_model_traverses_one_observation_to_original_closing(monkeypatch, tmp_path, order):
    monkeypatch.setattr(chain, 'ray_scene', cuboid_scene)
    c = chain.unified_replay(monkeypatch, tmp_path, order, perturb=True)
    from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence
    from alicia_flexible_grasp.vision.regular_box_inference import RegularBoxInference
    model = RegularBoxInference.from_evidence_dict(
        decode_contact_evidence(c.contact, required=True)['box_model_evidence'])
    assert model.schema_version == 3
    assert model.source['stamp_ns'] == c.current_row['object'].header.stamp.to_nsec()
    assert model.qualification['contour_source'] == 'cuboid_rgb_joint_interval_on_measured_support'
    # Preserve the original executable plan and its geometry across admission,
    # task freeze, approach and close. A new source cannot be invented here.
    from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
    assert compute_plan_id(c.contact) == c.contact.plan_id
    assert c.contact.valid
    assert [r['state'] for r in c.requests] == ['executing', 'requalifying', 'qualified']
    observation = [r for r in c.device_calls if r[0] == 'motion' and r[1] == c.origin.plan_id]
    contact = [r for r in c.device_calls if r[0] == 'motion' and r[1] == c.contact.plan_id]
    closes = [r for r in c.device_calls if r[0] == 'gripper' and r[2] < .05]
    assert len(observation) == 1 and len(contact) == 3 and len(closes) == 1
    assert c.execution_result is False
    assert c.states[-1].state == 'HOLDING'
    assert 'GRASP_HOLD_UNVERIFIED' in c.states[-1].message
    assert not any(r[0] == 'lift' for r in c.device_calls)
